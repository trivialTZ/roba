"""Generate a synthetic meat-cutting video to exercise the video2traj pipeline end-to-end (Aim 2.3).

Real footage needs dataset access (EPIC-KITCHENS / YouCook2 for food, Cholec80 for surgical — see
docs/REFERENCES.md). To validate the pipeline reproducibly here, we synthesize a clip: a dark knife
performs downward press + lateral slice strokes (a press-push-slice motion) over a 3-layer slab
(skin/fat/lean bands). The pipeline must recover this known trajectory — so we also dump the ground
truth for comparison.

Usage:  python perception/make_synthetic_cut_video.py [out.mp4|out.avi]
"""
import json
import os
import sys

import cv2
import numpy as np

W, H, FPS, N = 480, 360, 30, 150
OUT = sys.argv[1] if len(sys.argv) > 1 else "perception/synthetic_cut.avi"


def main():
    os.makedirs(os.path.dirname(OUT) or ".", exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")  # AVI/MJPG: no ffmpeg needed
    vw = cv2.VideoWriter(OUT, fourcc, FPS, (W, H))
    gt = []  # ground-truth knife-tip pixel path (normalized)

    # 3 horizontal tissue bands (lean bottom, fat mid, skin top) for visual context
    bands = [(int(H * 0.55), H, (90, 80, 200)),       # lean (BGR reddish)
             (int(H * 0.42), int(H * 0.55), (210, 230, 245)),  # fat (cream)
             (int(H * 0.34), int(H * 0.42), (180, 205, 225))]  # skin (tan)

    n_stations, per = 3, N // 3
    for i in range(N):
        frame = np.full((H, W, 3), 235, np.uint8)
        for y0, y1, col in bands:
            cv2.rectangle(frame, (60, y0), (W - 60, y1), col, -1)
        cv2.rectangle(frame, (40, int(H * 0.78)), (W - 40, int(H * 0.86)), (60, 90, 120), -1)  # board

        st = min(i // per, n_stations - 1)
        f = (i % per) / per
        x_station = 0.30 + 0.20 * st                       # station x (normalized)
        slice_dx = 0.03 * np.sin(2 * np.pi * 2.0 * (i / FPS))  # lateral slicing oscillation
        kx = x_station + slice_dx
        ky = 0.30 + 0.50 * f                               # press downward over the stroke
        px, py = int(kx * W), int(ky * H)

        # draw the knife: blade (dark rect) + handle
        cv2.rectangle(frame, (px - 3, py - 70), (px + 3, py), (40, 40, 40), -1)
        cv2.rectangle(frame, (px - 7, py - 95), (px + 7, py - 70), (30, 30, 30), -1)
        vw.write(frame)
        gt.append({"frame": i, "knife_tip_norm": [round(kx, 4), round(ky, 4)],
                   "station": st, "phase": "slice" if f > 0.15 else "approach"})

    vw.release()
    json.dump({"fps": FPS, "w": W, "h": H, "ground_truth": gt},
              open(os.path.splitext(OUT)[0] + "_gt.json", "w"), indent=1)
    print(f"wrote {OUT} ({N} frames) + ground truth")


if __name__ == "__main__":
    main()
