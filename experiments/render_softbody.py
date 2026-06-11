"""Render the deformable Warp cut (warp_cut.json) — driver-free VTK, shows squish + separation.

Builds hexahedral cells from the lattice node positions each frame, colored by tissue layer, with
smooth shading + meat materials. Cells that have torn apart (any edge stretched well beyond rest) are
DROPPED, so the cut shows as a clean gap in a solid, deforming block. Off-screen VTK → no GPU driver.

    .venv/bin/python experiments/render_softbody.py experiments/out/warp_cut.json [--still N]
"""
import json
import os
import sys

import numpy as np
import pyvista as pv

pv.OFF_SCREEN = True

FIN = {0: dict(diffuse=0.9, specular=0.35), 1: dict(diffuse=0.9, specular=0.45), 2: dict(diffuse=0.9, specular=0.3)}


def main():
    rec = sys.argv[1] if len(sys.argv) > 1 else "experiments/out/warp_cut.json"
    base = os.path.splitext(rec)[0]
    still = int(sys.argv[sys.argv.index("--still") + 1]) if "--still" in sys.argv else None
    d = json.load(open(rec)); M = d["meta"]; frames = d["frames"]
    NX, NY, NZ, DX = M["NX"], M["NY"], M["NZ"], M["DX"]
    node_layer = np.array(M["node_layer"]); colors = M["layer_colors"]
    print(f"loaded {len(frames)} frames, {M['n_nodes']} nodes, broken {M['springs_broken']}/{M['n_springs']}")

    def nid(i, j, k): return (i * NY + j) * NZ + k
    # cells: (8 corner node ids, layer) for each lattice cell
    cells = []
    for i in range(NX - 1):
        for j in range(NY - 1):
            for k in range(NZ - 1):
                c = [nid(i,j,k), nid(i+1,j,k), nid(i+1,j+1,k), nid(i,j+1,k),
                     nid(i,j,k+1), nid(i+1,j,k+1), nid(i+1,j+1,k+1), nid(i,j+1,k+1)]
                lay = int(node_layer[nid(i,j,k+1)])   # color by upper layer of the cell
                cells.append((c, lay))
    tear2 = (1.9 * DX) ** 2   # squared edge length above which a cell is "cut"

    def build(P):
        # group quads (cube faces) by layer, skipping torn cells; PolyData per layer for coloring
        faces_by = {0: [], 1: [], 2: []}
        FACE = [(0,1,2,3),(4,5,6,7),(0,1,5,4),(2,3,7,6),(0,3,7,4),(1,2,6,5)]
        for c, lay in cells:
            cp = P[c]
            e = cp[[1,2,3,0,5,6,7,4]] - cp[[0,1,2,3,4,5,6,7]]
            if (np.sum(e*e, axis=1).max()) > tear2:   # torn -> skip (reveals the cut)
                continue
            for f in FACE:
                faces_by[lay].append(cp[list(f)])
        return faces_by

    pts0 = np.array(frames[0]).reshape(-1, 3)
    ctr = pts0.mean(0); rad = (pts0.max(0) - pts0.min(0)).max() * 1.5
    cam = (ctr + np.array([rad*1.3, -rad*1.7, rad*1.0]), ctr, (0, 0, 1))

    def draw(plotter, fi):
        P = np.array(frames[fi]).reshape(-1, 3)
        fb = build(P)
        for lay, quads in fb.items():
            if not quads: continue
            n = len(quads)
            pad = np.full((n, 1), 4)
            faces = np.hstack([pad, np.arange(n*4).reshape(n, 4)]).ravel()
            poly = pv.PolyData(np.array(quads).reshape(-1, 3), faces)
            plotter.add_mesh(poly, color=colors[lay], smooth_shading=False,
                             diffuse=FIN[lay]["diffuse"], specular=FIN[lay]["specular"],
                             specular_power=15, ambient=0.28, show_edges=False)
        bd = pv.Cube(center=(ctr[0], ctr[1], pts0.min(0)[2]-0.012), x_length=rad*1.4,
                     y_length=rad*1.2, z_length=0.02)
        plotter.add_mesh(bd, color=(0.45, 0.30, 0.18), ambient=0.3)
        # knife marker (a thin steel plate near the active cut x — approximate from frame fraction)
        plotter.camera_position = cam

    if still is not None:
        p = pv.Plotter(off_screen=True, window_size=(900, 700), lighting="three lights")
        p.set_background("white"); draw(p, still); p.screenshot(base + f"_f{still}.png"); p.close()
        print("wrote", base + f"_f{still}.png"); return

    pl = pv.Plotter(off_screen=True, window_size=(900, 700), lighting="three lights")
    pl.open_gif(base + ".gif", fps=12)
    for fi in range(len(frames)):
        pl.clear(); pl.set_background("white"); draw(pl, fi); pl.write_frame()
    pl.close(); print("wrote", base + ".gif")


if __name__ == "__main__":
    main()
