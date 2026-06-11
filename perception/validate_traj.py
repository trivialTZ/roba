"""Validate the extracted trajectory against ground truth (Aim 2.3 result + figure).

Compares the video2traj-extracted knife-tip path to the synthetic clip's ground truth, reports a
tracking error, and plots both. Demonstrates the perception pipeline recovers a usable KINEMATIC
model from video (the realistic half of 2.3; forces remain sensor/sim-derived — ADR-005).
"""
import json
import os

import numpy as np


def main():
    prior = json.load(open("experiments/out/trajectory_prior.json"))
    gt = json.load(open("perception/synthetic_cut_gt.json"))["ground_truth"]
    samples = prior["samples"]

    # align by nearest frame index (GT is per-frame; samples are subsampled at sample_fps)
    gt_xy = {g["frame"]: g["knife_tip_norm"] for g in gt}
    src_fps, samp_fps = gt and 30, prior["fps"]
    stride = max(1, int(round(30 / prior["fps"])))

    ex, tgt = [], []
    for k, s in enumerate(samples):
        gframe = k * stride
        if gframe in gt_xy and s["flow_mag"] > 0:  # skip the first (no-flow) sample
            ex.append(s["tool_xy"]); tgt.append(gt_xy[gframe])
    ex, tgt = np.array(ex), np.array(tgt)
    err = np.linalg.norm(ex - tgt, axis=1)
    result = {
        "n_matched": len(ex),
        "mean_norm_error": round(float(err.mean()), 4),
        "median_norm_error": round(float(np.median(err)), 4),
        "extracted_slice_push": prior.get_slice_push if False else None,
    }
    print("=== Aim 2.3 trajectory validation ===")
    print(f" matched samples: {len(ex)}")
    print(f" mean normalized tracking error:   {err.mean():.3f}")
    print(f" median normalized tracking error: {np.median(err):.3f}")
    json.dump(result, open("experiments/out/traj_validation.json", "w"), indent=2)

    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 5))
        ax.plot(tgt[:, 0], tgt[:, 1], "-o", color="#6f7479", ms=3, label="ground truth")
        ax.plot(ex[:, 0], ex[:, 1], "-x", color="#b3514f", ms=4, label="extracted (optical flow)")
        ax.invert_yaxis()  # image coords: y down
        ax.set_xlabel("x (norm)"); ax.set_ylabel("y (norm, image)")
        ax.set_title(f"Aim 2.3: knife trajectory from video\nmean tracking error = {err.mean():.3f} (normalized)")
        ax.legend(); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig("experiments/out/traj_validation.png", dpi=130)
        print(" wrote experiments/out/traj_validation.png")
    except Exception as e:
        print(f" (plot skipped: {e})")


if __name__ == "__main__":
    main()
