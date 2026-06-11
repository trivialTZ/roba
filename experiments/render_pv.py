"""Rasterized replay renderer (PyVista/VTK) — driver-agnostic, proper lighting + meat materials.

Upgrade over the flat-matplotlib renderer: VTK rasterization with PBR-ish materials, smooth shading,
no edge lines (so adjacent same-layer blocks read as a continuous slab rather than bricks), and lights
for a wet-meat sheen. Needs NO NVIDIA driver / Vulkan / RTX — VTK uses CPU/software or native GL.

    .venv/bin/python experiments/render_pv.py experiments/out/recording_slice.json [--still N]

Reads the recording (roba_sim.recording format): per box per frame = [tx,ty,tz, m00..m22] (12 floats).
"""
import json
import os
import sys

import numpy as np
import pyvista as pv

pv.OFF_SCREEN = True

# unit-cube corners (±0.5) and quad faces in VTK format [4,i,j,k,l,...]
_C = np.array([[sx, sy, sz] for sx in (-0.5, 0.5) for sy in (-0.5, 0.5) for sz in (-0.5, 0.5)])
_FACES = np.hstack([[4, *f] for f in [(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)]])

# meat-like materials per layer name fragment (RGB 0..1)
MAT = {
    "lean": dict(color=(0.62, 0.16, 0.18), roughness=0.55, metallic=0.0, diffuse=0.9, specular=0.35),
    "fat":  dict(color=(0.96, 0.93, 0.82), roughness=0.45, metallic=0.0, diffuse=0.9, specular=0.45),
    "skin": dict(color=(0.86, 0.72, 0.56), roughness=0.6,  metallic=0.0, diffuse=0.9, specular=0.3),
    "blade":dict(color=(0.80, 0.82, 0.86), roughness=0.2,  metallic=0.8, diffuse=0.6, specular=0.9),
}


def mat_for(name):
    for k in MAT:
        if k in name:
            return MAT[k]
    return MAT["lean"]


def corners(entry):
    t = np.array(entry[0:3]); M3 = np.array(entry[3:12]).reshape(3, 3)
    return _C @ M3 + t


def mesh(entry):
    return pv.PolyData(corners(entry), _FACES)


def finish_for(color):
    """Pick material finish from the recorded layer color (cream=fat, tan=skin, steel=blade, else lean)."""
    r, g, b = color
    if r > 0.7 and g > 0.78 and b < 0.92 and b > 0.7:      # steel-ish gray-blue (blade)
        return dict(roughness=0.2, diffuse=0.6, specular=0.9)
    if r > 0.9 and g > 0.88:                                # cream (fat)
        return dict(roughness=0.45, diffuse=0.9, specular=0.45)
    if 0.8 < r < 0.92 and 0.6 < g < 0.8:                    # tan (skin)
        return dict(roughness=0.6, diffuse=0.9, specular=0.3)
    return dict(roughness=0.55, diffuse=0.9, specular=0.35)  # red (lean) default


def main():
    rec = sys.argv[1] if len(sys.argv) > 1 else "experiments/out/recording_slice.json"
    base = os.path.splitext(rec)[0] + "_pv"
    still = None
    if "--still" in sys.argv:
        still = int(sys.argv[sys.argv.index("--still") + 1])
    data = json.load(open(rec)); meta, frames = data["meta"], data["frames"]
    cols = [m["color"] for m in meta]   # USE the recorded per-layer colors (skin/fat/lean), not the name
    print(f"loaded {len(frames)} frames x {len(meta)} boxes")

    pts0 = np.vstack([corners(b) for b in frames[0]])
    ctr = pts0.mean(0); rad = (pts0.max(0) - pts0.min(0)).max() * 1.5
    cam = (ctr + np.array([rad*1.4, -rad*1.8, rad*1.1]), ctr, (0, 0, 1))

    def render(fi, fname):
        p = pv.Plotter(off_screen=True, window_size=(900, 700), lighting="three lights")
        p.set_background("white")
        for bi, e in enumerate(frames[fi]):
            f = finish_for(cols[bi])
            p.add_mesh(mesh(e), color=cols[bi], smooth_shading=True,
                       diffuse=f["diffuse"], specular=f["specular"], specular_power=18,
                       ambient=0.25, show_edges=False)
        # cutting board
        b = pv.Cube(center=(ctr[0], ctr[1], pts0.min(0)[2]-0.012), x_length=rad*1.3,
                    y_length=rad*1.1, z_length=0.02)
        p.add_mesh(b, color=(0.45, 0.30, 0.18), smooth_shading=True, ambient=0.3)
        p.camera_position = cam
        p.screenshot(fname)
        p.close()

    if still is not None:
        render(still, base + f"_f{still}.png"); print("wrote", base + f"_f{still}.png"); return

    gif = base + ".gif"
    pl = pv.Plotter(off_screen=True, window_size=(900, 700), lighting="three lights")
    pl.open_gif(gif, fps=12)
    for fi in range(len(frames)):
        pl.clear()
        pl.set_background("white")
        for bi, e in enumerate(frames[fi]):
            f = finish_for(cols[bi])
            pl.add_mesh(mesh(e), color=cols[bi], smooth_shading=True,
                        diffuse=f["diffuse"], specular=f["specular"], specular_power=18,
                        ambient=0.25, show_edges=False)
        bd = pv.Cube(center=(ctr[0], ctr[1], pts0.min(0)[2]-0.012), x_length=rad*1.3,
                     y_length=rad*1.1, z_length=0.02)
        pl.add_mesh(bd, color=(0.45, 0.30, 0.18), ambient=0.3)
        pl.camera_position = cam
        pl.write_frame()
    pl.close()
    print("wrote", gif)


if __name__ == "__main__":
    main()
