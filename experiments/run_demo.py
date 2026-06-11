"""Entry point for the interactive Aim-1 demo (Aim 1.1–1.5).

Run INSIDE the Isaac Sim container (deploy/scc/run_interactive.sh swaps runheadless.sh for this),
or locally via:  ./python.sh experiments/run_demo.py  [--headless]

The SimulationApp MUST be created before any isaacsim/omni imports — that's why this file builds
it first, then imports roba_sim. Connect from your Mac via SCC OnDemand desktop or WebRTC.
"""
import argparse
import sys

# 1) Bootstrap the simulator FIRST (order matters).
try:
    from isaacsim import SimulationApp  # 5.x/6.x
except Exception:
    from omni.isaac.kit import SimulationApp  # ≤4.x


def main() -> int:
    parser = argparse.ArgumentParser(description="roba meat-cutting demo")
    parser.add_argument("--headless", action="store_true", help="no GUI (SCC batch)")
    parser.add_argument("--auto", choices=["skive", "slice"], default=None,
                        help="run an autonomous task instead of waiting for the mouse")
    args = parser.parse_args()

    sim_app = SimulationApp({"headless": args.headless})

    # 2) Now safe to import everything that touches Isaac Sim.
    sys.path.insert(0, "/workspace/roba/src")  # repo layout inside the container
    from roba_sim.config import default_config
    from roba_sim.app import MODE_SKIVE, MODE_SLICE, RobaApp

    cfg = default_config()
    cfg.sim.headless = args.headless

    app = RobaApp(cfg, sim_app)
    app.setup()

    if args.auto == "skive":
        app.set_mode(MODE_SKIVE); app.start()
    elif args.auto == "slice":
        app.set_mode(MODE_SLICE); app.start()

    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
