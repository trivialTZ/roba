"""Render figures from the headless experiment results (paper artifacts).

Reads experiments/out/*.json and writes PNGs. Run in an env with matplotlib:
    python -m venv .venv && .venv/bin/pip install matplotlib && .venv/bin/python experiments/plot_results.py
"""
import json
import os

HERE = os.path.dirname(__file__)
OUT = os.path.join(HERE, "out")


def load(name):
    p = os.path.join(OUT, name)
    return json.load(open(p)) if os.path.exists(p) else None


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    # --- Force-based cutting: configured vs measured break force (log scale) ---
    fc = load("force_cut_results.json")
    if fc and fc.get("layers"):
        layers = ["lean", "fat", "skin"]
        conf = [fc["layers"][l]["configured_break_force_N"] for l in layers]
        meas = [fc["layers"][l]["measured_break_force_N"] for l in layers]
        x = np.arange(len(layers))
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.bar(x - 0.2, conf, 0.4, label="configured (material.py)", color="#9aa0a6")
        ax.bar(x + 0.2, meas, 0.4, label="measured (PhysX)", color="#b3514f")
        ax.set_yscale("log")
        ax.set_xticks(x); ax.set_xticklabels(layers)
        ax.set_ylabel("break force (N, log)")
        ax.set_title("Force to sever each layer: model vs PhysX\n(skin ≫ fat > lean)")
        ax.legend()
        for xi, (c, m) in enumerate(zip(conf, meas)):
            ax.text(xi + 0.2, m * 1.05, f"{m:.0f}", ha="center", fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "force_break.png"), dpi=130)
        print("wrote force_break.png")

    # --- Skin-skiving selectivity ---
    sk = load("skin_skive_results.json")
    if sk and sk.get("summary"):
        s = sk["summary"]
        cats = ["skin/fat\nsevered", "lean/fat\nsevered", "skin cols\nsevered"]
        vals = [s["skin_fat_interfaces_severed"], s["lean_fat_interfaces_severed"],
                s["skin_column_seams_severed"]]
        tot = s["skin_fat_interfaces_total"]
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.bar(cats, vals, color=["#b3514f", "#6f7479", "#6f7479"], edgecolor="k")
        ax.axhline(tot, ls="--", color="#b3514f", alpha=0.6, label=f"all skin/fat = {tot}")
        ax.set_ylabel("seams severed")
        ax.set_title(f"Skin-skiving selectivity (peel fraction = {s['skin_peel_fraction']})\n"
                     f"whole skin sheet off, deeper tissue + sheet intact")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "skive_selectivity.png"), dpi=130)
        print("wrote skive_selectivity.png")

    # --- IK-driven dual-arm cut: knife descent + cut progression ---
    ik = load("dual_arm_ik_cut_results.json")
    if ik and ik.get("trajectory"):
        traj = ik["trajectory"]
        step = [t["step"] for t in traj]
        kz = [t["knife"][2] for t in traj]
        frac = [t["fraction_cut"] for t in traj]
        fig, ax1 = plt.subplots(figsize=(6.5, 4))
        ax1.plot(step, kz, "-o", color="#3b6ea5", ms=4, label="knife height z (m)")
        ax1.set_xlabel("descent step"); ax1.set_ylabel("knife height z (m)", color="#3b6ea5")
        ax1.tick_params(axis="y", labelcolor="#3b6ea5")
        ax2 = ax1.twinx()
        ax2.plot(step, frac, "-s", color="#b3514f", ms=4, label="fraction cut")
        ax2.set_ylabel("fraction of seams cut", color="#b3514f")
        ax2.tick_params(axis="y", labelcolor="#b3514f")
        ax1.set_title("IK-driven dual-arm cut (Aim 1.3)\nknife descends → seams break as it crosses the slab")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "ik_cut.png"), dpi=130)
        print("wrote ik_cut.png")

    print("done")


if __name__ == "__main__":
    main()
