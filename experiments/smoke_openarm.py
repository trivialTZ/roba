"""Minimal Isaac Sim smoke test (Phase 0.1).

Validates, in one headless run: (1) the container boots Isaac Sim, (2) the OpenArm USD loads,
(3) we can introspect the articulation to learn the REAL link/joint names (so we can fix
config.OPENARM_CUTTING.ee_frame). Prints lots of diagnostics and never hard-crashes — the point is
to learn the live API behavior on SCC. Run via the smoke batch job (headless).
"""
import os
import sys
import traceback

# 1) Boot the simulator FIRST.
try:
    from isaacsim import SimulationApp
except Exception:
    from omni.isaac.kit import SimulationApp

# On a multi-GPU node SGE sets CUDA_VISIBLE_DEVICES to our 1 allocated GPU, but Vulkan still sees
# ALL GPUs → Isaac can't match Vulkan↔CUDA and vkCreateDevice fails. Map the renderer's active_gpu
# to the allocated physical index (the value in CUDA_VISIBLE_DEVICES); physics uses CUDA's view (0).
_cvd = os.environ.get("CUDA_VISIBLE_DEVICES", "").split(",")[0].strip()
_active = int(_cvd) if _cvd.isdigit() else 0
print(f"[smoke] CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES')!r} -> active_gpu={_active}", flush=True)
sim = SimulationApp({
    "headless": True,
    "active_gpu": _active,     # Vulkan/render device (full enumeration index)
    "physics_gpu": 0,          # CUDA device index (restricted set → 0)
    "multi_gpu": False,
})

def banner(s): print(f"\n========== {s} ==========", flush=True)

try:
    import isaacsim
    banner("isaacsim version")
    print("isaacsim:", getattr(isaacsim, "__version__", "unknown"), flush=True)
except Exception as e:
    print("no isaacsim module:", e, flush=True)

sys.path.insert(0, "/workspace/roba/src")
from roba_sim import _isaac_compat as ic

banner("compat shim")
print("ISAAC_AVAILABLE:", ic.ISAAC_AVAILABLE, flush=True)

asset_root = os.environ.get("ROBA_ASSET_ROOT", "/workspace/assets/robots")
usd = f"{asset_root}/openarm_isaac_lab/source/openarm/openarm/tasks/manager_based/openarm_manipulation/usds/openarm_unimanual/openarm_unimanual.usd"
banner("asset")
print("USD:", usd, "\nexists:", os.path.exists(usd), flush=True)

try:
    banner("build world + load OpenArm USD")
    world = ic.World()
    ic.add_reference_to_stage(usd_path=usd, prim_path="/World/OpenArm")
    world.reset()
    print("world built; USD referenced under /World/OpenArm", flush=True)

    banner("stage prims under /World/OpenArm (link names)")
    stage = ic.get_current_stage()
    links, joints = [], []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if p.startswith("/World/OpenArm"):
            t = prim.GetTypeName()
            if "Joint" in str(t):
                joints.append(p)
            elif t in ("Xform", "Mesh"):
                links.append((p, str(t)))
    print(f"#prims under OpenArm: links/xforms={len(links)}, joints={len(joints)}", flush=True)
    print("first 40 link/xform prims:", flush=True)
    for p, t in links[:40]:
        print("  ", t, p, flush=True)
    print("joint prims (first 30):", flush=True)
    for p in joints[:30]:
        print("  ", p, flush=True)

    banner("articulation introspection")
    try:
        art = (ic.Articulation(prim_paths_expr="/World/OpenArm")
               if "prim_paths_expr" in ic.Articulation.__init__.__code__.co_varnames
               else ic.Articulation(prim_path="/World/OpenArm"))
        world.scene.add(art) if hasattr(world.scene, "add") else None
        world.reset()
        for _ in range(3):
            world.step(render=False)
        print("dof_names:", getattr(art, "dof_names", "n/a"), flush=True)
        print("num_dof:", getattr(art, "num_dof", "n/a"), flush=True)
        try:
            print("body_names:", art.body_names, flush=True)
        except Exception as e:
            print("body_names unavailable:", e, flush=True)
    except Exception:
        print("Articulation introspection failed:", flush=True)
        traceback.print_exc()

    print("\nSMOKE OK", flush=True)
except Exception:
    print("SMOKE FAILED:", flush=True)
    traceback.print_exc()
finally:
    sim.close()
