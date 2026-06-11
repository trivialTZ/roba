"""Driver-agnostic replay renderer (Tier-1 visualization).

Reads a recording (roba_sim.recording.Recorder output) and renders the boxes frame-by-frame with
matplotlib 3D — no NVIDIA driver / Isaac renderer needed. Writes an animated GIF (PillowWriter,
always available with matplotlib) and an MP4 if ffmpeg is present. Runs anywhere, e.g. the Mac:

    .venv/bin/python experiments/render_replay.py experiments/out/recording_slice.json

Box geometry: each prim is a unit cube (corners ±0.5); world corner = local @ M3 + t, where (t, M3)
are the 12 floats recorded per box per frame (see roba_sim/recording.py).
"""
import json
import os
import sys

import numpy as np

# unit-cube corners (UsdGeom.Cube size=1 spans -0.5..0.5) and its 6 quad faces
_C = np.array([[sx, sy, sz] for sx in (-0.5, 0.5) for sy in (-0.5, 0.5) for sz in (-0.5, 0.5)])
_FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]


def box_world_corners(entry):
    t = np.array(entry[0:3])
    M3 = np.array(entry[3:12]).reshape(3, 3)
    return _C @ M3 + t  # (8,3)


def main():
    rec_path = sys.argv[1] if len(sys.argv) > 1 else "experiments/out/recording_slice.json"
    out_base = os.path.splitext(rec_path)[0]
    data = json.load(open(rec_path))
    meta, frames = data["meta"], data["frames"]
    colors = [m["color"] for m in meta]
    print(f"loaded {len(frames)} frames x {len(meta)} boxes from {rec_path}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    # Frame the camera on the INTACT slab (first frame). Freed pieces can get a large impulse from the
    # kinematic blade and fly off — auto-fitting to that debris would shrink the slab to a dot. We bound
    # to the initial slab + padding; pieces that leave the box just exit frame.
    pts0 = np.array([box_world_corners(b) for b in frames[0]]).reshape(-1, 3)
    lo, hi = pts0.min(0), pts0.max(0)
    ctr, rad = (lo + hi) / 2, (hi - lo).max() / 2 * 1.6

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    def draw(fi):
        ax.clear()
        fr = frames[fi]
        polys, facecolors = [], []
        for bi, entry in enumerate(fr):
            cor = box_world_corners(entry)
            for f in _FACES:
                polys.append([cor[i] for i in f])
                facecolors.append(colors[bi])
        pc = Poly3DCollection(polys, facecolors=facecolors, edgecolors=(0, 0, 0, 0.25), linewidths=0.3)
        ax.add_collection3d(pc)
        ax.set_xlim(ctr[0] - rad, ctr[0] + rad)
        ax.set_ylim(ctr[1] - rad, ctr[1] + rad)
        ax.set_zlim(ctr[2] - rad, ctr[2] + rad)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=18, azim=-60)
        ax.set_title(f"roba meat cut — frame {fi+1}/{len(frames)}")
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")

    from matplotlib import animation
    anim = animation.FuncAnimation(fig, draw, frames=len(frames), interval=80)

    gif = out_base + ".gif"
    anim.save(gif, writer=animation.PillowWriter(fps=12))
    print(f"wrote {gif}")
    try:
        mp4 = out_base + ".mp4"
        anim.save(mp4, writer=animation.FFMpegWriter(fps=12, bitrate=2400))
        print(f"wrote {mp4}")
    except Exception as e:
        print(f"(mp4 skipped: {e} — GIF written)")

    # also a still of the final frame for the paper
    draw(len(frames) - 1)
    still = out_base + "_final.png"
    fig.savefig(still, dpi=130)
    print(f"wrote {still}")


if __name__ == "__main__":
    main()
