"""Plot the headless cut results (Aim 2.4b figure for the paper).

Reads experiments/out/headless_cut_results.json and writes cut_progress.png (cut fraction over the
blade sweep + per-layer seam breaks). Falls back to a text summary if matplotlib is unavailable.

Usage:  python experiments/plot_cut.py
"""
import json
import os

HERE = os.path.dirname(__file__)
RES = os.path.join(HERE, "out", "headless_cut_results.json")


def main():
    with open(RES) as f:
        data = json.load(f)
    steps = data["steps"]
    summ = data["summary"]

    print("=== headless cut summary ===")
    print(f" columns={summ['n_columns']} seams_total={summ['n_seams_total']} "
          f"broken={summ['n_seams_broken']} fraction_cut={summ['fraction_cut']}")
    print(" broken_by_layer:")
    for k, v in summ["broken_by_layer"].items():
        print(f"   {k:12s} {v}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"\n(matplotlib unavailable: {e} — install it to render the figure)")
        return

    xs = [s["step"] for s in steps]
    fc = [s["fraction_cut"] for s in steps]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4))

    ax1.plot(xs, fc, "-o", color="#b3514f", ms=4)
    for xstn in summ.get("x_stations", []):
        pass
    ax1.set_xlabel("physics step")
    ax1.set_ylabel("fraction of seams cut")
    ax1.set_title("Vertical slicing: cut progression\n(3 stations, blade descending)")
    ax1.grid(alpha=0.3)

    layers = list(summ["broken_by_layer"].keys())
    vals = [summ["broken_by_layer"][k] for k in layers]
    colors = {"skin": "#edd9c7", "fat": "#faf7e6", "lean": "#c75254",
              "lean/fat": "#9aa0a6", "fat/skin": "#6f7479"}
    ax2.bar(layers, vals, color=[colors.get(k, "#888") for k in layers], edgecolor="k")
    ax2.set_ylabel("seams broken")
    ax2.set_title("Seams cut, by layer / interface")
    ax2.tick_params(axis="x", rotation=30)

    fig.tight_layout()
    out = os.path.join(HERE, "out", "cut_progress.png")
    fig.savefig(out, dpi=130)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
