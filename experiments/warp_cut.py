"""Deformable layered meat cut in NVIDIA Warp (the 'feels like meat' path).

A mass-spring soft body — the deformable upgrade of our breakable-seam idea: lattice nodes that
SQUISH under the knife, connected by springs that BREAK when the blade passes or strain exceeds a
per-layer threshold (skin tougher than lean tougher than fat, from docs/MATERIAL_MODEL.md ordering).
Runs on CUDA via warp-lang (no Vulkan/RTX → unaffected by the 595 driver). Records node positions per
frame to JSON for driver-agnostic VTK rendering (render_softbody.py).

Stiffness is visual-scaled (kPa-range effective springs) for stable real-time explicit integration —
the relative layer ordering is preserved; absolute cutting forces live in the force_cut experiment.

Run on a GPU node via the wenv:  wenv/bin/python experiments/warp_cut.py
"""
import json
import os
import sys

import numpy as np
import warp as wp

wp.init()
OUT = os.environ.get("ROBA_OUT", "/tmp")

# ---- lattice / layers -----------------------------------------------------------------
NX, NY, NZ = 30, 10, 12          # nodes; long in X (cut dir), Z up
DX = 0.006                        # node spacing (m) -> block ~0.174 x 0.054 x 0.066 m
# layer by k (bottom->top): lean (0..6), fat (7..10), skin (11)
def layer_of_k(k):
    if k >= NZ - 1: return 2      # skin (top, 1 node thick)
    if k >= NZ - 4: return 1      # fat
    return 0                      # lean
LAYER_COL = [(0.62, 0.16, 0.18), (0.96, 0.93, 0.82), (0.86, 0.72, 0.56)]  # lean/fat/skin
# visual-scaled spring stiffness (N/m): soft (meat-like) and stable for explicit integration; skin>lean>fat
LAYER_STIFF = [700.0, 350.0, 1400.0]
LAYER_BREAK = [0.30, 0.22, 0.60]   # break strain: skin stretches most before tearing
LAYER_DENS = [1060.0, 920.0, 1100.0]

def nid(i, j, k): return (i * NY + j) * NZ + k
N = NX * NY * NZ

# node positions, layer, mass, pinned(bottom)
pos = np.zeros((N, 3), np.float32); node_layer = np.zeros(N, np.int32)
mass = np.zeros(N, np.float32); pinned = np.zeros(N, np.int32)
for i in range(NX):
    for j in range(NY):
        for k in range(NZ):
            n = nid(i, j, k)
            pos[n] = (i * DX, j * DX, 0.02 + k * DX)   # sit at z>=0.02
            lay = layer_of_k(k); node_layer[n] = lay
            mass[n] = max(LAYER_DENS[lay] * DX**3, 1e-4)
            if k == 0: pinned[n] = 1                     # bottom welded to board

# springs: structural (6-neigh) + face/shear diagonals for stability
pairs, srest, sstiff, sbreak = [], [], [], []
def add_spring(a, b):
    d = np.linalg.norm(pos[a] - pos[b]); la = max(node_layer[a], node_layer[b])
    pairs.append((a, b)); srest.append(d); sstiff.append(LAYER_STIFF[la]); sbreak.append(LAYER_BREAK[la])
offs = [(1,0,0),(0,1,0),(0,0,1),(1,1,0),(1,0,1),(0,1,1),(1,-1,0),(1,0,-1),(0,1,-1)]
for i in range(NX):
    for j in range(NY):
        for k in range(NZ):
            for di,dj,dk in offs:
                ii,jj,kk = i+di, j+dj, k+dk
                if 0<=ii<NX and 0<=jj<NY and 0<=kk<NZ:
                    add_spring(nid(i,j,k), nid(ii,jj,kk))
S = len(pairs)
print(f"[warp_cut] {N} nodes, {S} springs", flush=True)

# ---- warp arrays ----------------------------------------------------------------------
dev = "cuda"
x = wp.array(pos, dtype=wp.vec3, device=dev)
v = wp.zeros(N, dtype=wp.vec3, device=dev)
f = wp.zeros(N, dtype=wp.vec3, device=dev)
m = wp.array(mass, dtype=wp.float32, device=dev)
pin = wp.array(pinned, dtype=wp.int32, device=dev)
sa = wp.array(np.array([p[0] for p in pairs], np.int32), dtype=wp.int32, device=dev)
sb = wp.array(np.array([p[1] for p in pairs], np.int32), dtype=wp.int32, device=dev)
rest = wp.array(np.array(srest, np.float32), dtype=wp.float32, device=dev)
kst = wp.array(np.array(sstiff, np.float32), dtype=wp.float32, device=dev)
kbr = wp.array(np.array(sbreak, np.float32), dtype=wp.float32, device=dev)
broken = wp.zeros(S, dtype=wp.int32, device=dev)

@wp.kernel
def clear_f(f: wp.array(dtype=wp.vec3), m: wp.array(dtype=wp.float32)):
    i = wp.tid(); f[i] = wp.vec3(0.0, 0.0, -9.81 * m[i])   # gravity

@wp.kernel
def spring_f(x: wp.array(dtype=wp.vec3), sa: wp.array(dtype=wp.int32), sb: wp.array(dtype=wp.int32),
             rest: wp.array(dtype=wp.float32), kst: wp.array(dtype=wp.float32),
             broken: wp.array(dtype=wp.int32), f: wp.array(dtype=wp.vec3)):
    s = wp.tid()
    if broken[s] == 1: return
    a = sa[s]; b = sb[s]
    d = x[b] - x[a]; L = wp.length(d)
    if L < 1.0e-9: return
    dir = d / L
    force = kst[s] * (L - rest[s]) * dir
    wp.atomic_add(f, a, force); wp.atomic_add(f, b, -force)

@wp.kernel
def knife_contact(x: wp.array(dtype=wp.vec3), f: wp.array(dtype=wp.vec3),
                  kx: float, half_t: float, blade_z: float, kc: float):
    i = wp.tid(); p = x[i]
    # nodes within the blade's thin x-slab and above the blade edge get pushed apart in x + down
    if wp.abs(p[0] - kx) < half_t and p[2] > blade_z:
        side = 1.0
        if p[0] < kx: side = -1.0
        pen = half_t - wp.abs(p[0] - kx)
        wp.atomic_add(f, i, wp.vec3(side * kc * pen, 0.0, -kc * pen * 0.5))

@wp.kernel
def cut_springs(x: wp.array(dtype=wp.vec3), sa: wp.array(dtype=wp.int32), sb: wp.array(dtype=wp.int32),
                rest: wp.array(dtype=wp.float32), kbr: wp.array(dtype=wp.float32),
                broken: wp.array(dtype=wp.int32), kx: float, half_t: float, blade_z: float):
    s = wp.tid()
    if broken[s] == 1: return
    a = sa[s]; b = sb[s]
    mid = (x[a] + x[b]) * 0.5
    L = wp.length(x[b] - x[a]); strain = (L - rest[s]) / rest[s]
    # geometric cut: spring midpoint in the descended blade slab; OR over-strain tear
    if (wp.abs(mid[0] - kx) < half_t and mid[2] > blade_z) or strain > kbr[s]:
        broken[s] = 1

@wp.kernel
def integrate(x: wp.array(dtype=wp.vec3), v: wp.array(dtype=wp.vec3), f: wp.array(dtype=wp.vec3),
              m: wp.array(dtype=wp.float32), pin: wp.array(dtype=wp.int32), dt: float, damp: float,
              vmax: float):
    i = wp.tid()
    if pin[i] == 1:
        v[i] = wp.vec3(0.0, 0.0, 0.0); return
    nv = (v[i] + dt * f[i] / m[i]) * damp
    spd = wp.length(nv)
    if spd > vmax: nv = nv * (vmax / spd)   # runaway backstop
    v[i] = nv
    nx = x[i] + dt * nv
    if nx[2] < 0.0: nx = wp.vec3(nx[0], nx[1], 0.0)   # ground
    x[i] = nx

# ---- sim loop -------------------------------------------------------------------------
DT, SUB, FRAMES = 3.0e-5, 60, 110
KERF, KC, VMAX = DX * 0.9, 1500.0, 5.0
top_z = 0.02 + (NZ - 1) * DX
frames_out, max_speed = [], 0.0

def knife_schedule(frame):
    # three stations along X, descend at each (press-cut)
    per = FRAMES // 3; st = min(frame // per, 2); fpart = (frame % per) / per
    kx = DX * (NX * (0.25 + 0.25 * st))
    blade_z = top_z + 0.01 - (top_z + 0.01 - 0.021) * fpart
    return kx, blade_z

for frame in range(FRAMES):
    kx, blade_z = knife_schedule(frame)
    for _ in range(SUB):
        wp.launch(clear_f, dim=N, inputs=[f, m], device=dev)
        wp.launch(spring_f, dim=S, inputs=[x, sa, sb, rest, kst, broken, f], device=dev)
        wp.launch(knife_contact, dim=N, inputs=[x, f, kx, KERF, blade_z, KC], device=dev)
        wp.launch(integrate, dim=N, inputs=[x, v, f, m, pin, DT, 0.99, VMAX], device=dev)
    wp.launch(cut_springs, dim=S, inputs=[x, sa, sb, rest, kbr, broken, kx, KERF, blade_z], device=dev)
    wp.synchronize()
    xp = x.numpy()
    sp = float(np.linalg.norm(v.numpy(), axis=1).max()); max_speed = max(max_speed, sp)
    if not np.isfinite(xp).all():
        print(f"[warp_cut] NaN at frame {frame} — unstable", flush=True); break
    if frame % 2 == 0:
        frames_out.append(xp.reshape(-1).round(5).tolist())

nbroken = int(broken.numpy().sum())
meta = {"NX": NX, "NY": NY, "NZ": NZ, "DX": DX, "n_nodes": N, "n_springs": S,
        "node_layer": node_layer.tolist(), "layer_colors": LAYER_COL,
        "n_frames": len(frames_out), "springs_broken": nbroken, "max_speed": round(max_speed, 3),
        "stable": bool(max_speed < 50.0)}
os.makedirs(OUT, exist_ok=True)
json.dump({"meta": meta, "frames": frames_out}, open(os.path.join(OUT, "warp_cut.json"), "w"))
print(f"[warp_cut] frames={len(frames_out)} broken={nbroken}/{S} max_speed={max_speed:.2f} "
      f"stable={meta['stable']}", flush=True)
print("WARP_CUT_OK" if meta["stable"] and len(frames_out) > 0 else "WARP_CUT_UNSTABLE", flush=True)
