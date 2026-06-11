# roba — Robotic Meat-Cutting Simulation & Control

A dual-arm robotic **meat-cutting** simulation in NVIDIA Isaac Sim, paired with a data-driven control
and modeling study. Primary use case: meat cutting (pork belly); long-term aim: transferable findings
toward surgical cutting/resection.

> **Status:** pre-implementation. The technical plan has been researched and adversarially verified
> (see `docs/`). **Read `docs/FEASIBILITY.md` first** — it documents two spec assumptions that don't
> hold and how the plan re-scopes around them.

## The three Aims (summary)

- **Aim 1 — Interactive sim.** Isaac Sim; import two open arms (**OpenArm** by Enactic + **reBot Arm
  B601** by Seeed); 3-layer pork-belly material model (skin/fat/lean); dual-arm setup (one holds via
  impedance, one cuts); mouse-driven, plane-constrained IK; an `omni.ui` control panel.
- **Aim 2 — Data + modeling.** Survey cutting control theory (food + surgical); roles of force /
  impedance / visual servoing; **video → kinematic model** + **sensor/sim → force model**; control
  models for (a) skin skiving and (b) vertical slicing.
- **Aim 3 — Integration + paper.** Merge Aim 1 & 2; document; public repo + cs.RO-style PDF.

## ⚠️ Two verified re-scopes (don't skip)

1. **Isaac Sim cannot cut deformables natively.** No tearing/fracture API; NVIDIA says "not supported";
   the famous demos are breakable-joint fakes. → We use **pre-scored breakable seams** for the
   interactive demo and **DiSECt** (differentiable cutting sim) for the physics-faithful model.
   (`docs/ARCHITECTURE.md` ADR-001.)
2. **You can't get cutting *forces* from RGB video.** Trajectories yes, forces no (occluded blade,
   ill-posed). → Video gives the **kinematic/strategy** prior; forces come from a **real F/T sensor +
   sim calibration**. (ADR-005.)

Plus a **hardware gate**: Isaac Sim needs a dedicated RTX GPU with RT cores (min RTX 4080 / 16 GB VRAM);
A100/H100 are unsupported. **This project runs on BU SCC A40/A6000 nodes** (RT-core, supported) via
Apptainer; **macOS cannot host Isaac Sim**, so the Mac is a thin client (VNC/WebRTC). See ADR-008.

## Documentation

| Doc | What |
|-----|------|
| [`docs/FEASIBILITY.md`](docs/FEASIBILITY.md) | Pillar-by-pillar verdicts + verified findings (**start here**) |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Binding decisions (ADR-001…008) + system diagram |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | Phased plan (Phase 0 de-risking first) + risk register |
| [`docs/MATERIAL_MODEL.md`](docs/MATERIAL_MODEL.md) | Pork-belly skin/fat/lean parameter priors |
| [`docs/CONTROL_SURVEY.md`](docs/CONTROL_SURVEY.md) | Aim 2.1/2.2 control-theory synthesis |
| [`docs/REFERENCES.md`](docs/REFERENCES.md) | 160 annotated sources, by pillar |

## Repository layout

```
src/roba_sim/
  config.py          # all params as dataclasses (single source of truth, no GPU deps)
  _isaac_compat.py   # isaacsim.* vs omni.isaac.* namespace shim
  app.py             # RobaApp: wires the Aim-1 loop (teleop→IK→arms→cut→UI)
  cutting/           # material.py (toughness→break-force) + breakable_seams.py (ADR-001)
  scene/             # arms.py (OpenArm+reBot+knife), pork_belly.py, world.py
  control/           # impedance_hold.py, cutting_controller.py (press-push-slice, 2.4 tasks)
  teleop/            # mouse_plane.py (carb.input), plane_ik.py (Lula, plane-constrained)
  ui/                # control_panel.py (omni.ui sliders/buttons)
perception/video2traj/  # video → kinematic trajectory prior (ADR-005; forces NOT from RGB)
experiments/         # run_demo.py (entry point), ENV.md (pinned versions)
tests/               # test_core.py — GPU-free physics/control tests (run anywhere)
deploy/scc/          # BU SCC run kit + SCC_USAGE.md (don't-get-banned guide)
assets/robots/       # self-authored USD (no NVIDIA assets re-hosted)
paper/               # main.tex + outline.md (cs.RO-style)
docs/                # planning docs (FEASIBILITY, ARCHITECTURE, ROADMAP, CONTROL_SURVEY, …)
```

**Running on BU SCC:** see [`deploy/scc/README.md`](deploy/scc/README.md), and read
[`deploy/scc/SCC_USAGE.md`](deploy/scc/SCC_USAGE.md) **first** — the don't-get-banned guide (no
login-node compute; keep big files off the 10 GB home; never park an idle GPU).

## Getting started

**Now, on any machine (no GPU)** — validate the version-agnostic core (cutting physics, control/
teleop math):
```bash
python3 tests/test_core.py        # or: pip install -e ".[dev]" && pytest
```

**On BU SCC** — the Aim-1 simulation is implemented (`src/roba_sim/`, entry `experiments/run_demo.py`):
```bash
# inside a GPU job, in the Isaac Sim container (see deploy/scc/):
./python.sh experiments/run_demo.py            # interactive mouse-driven cutting
./python.sh experiments/run_demo.py --auto slice   # autonomous vertical slicing (Aim 2.4b)
```
First complete `docs/ROADMAP.md` **Phase 0** (pin Isaac Sim version in `experiments/ENV.md`, run the
two cutting spikes). The Isaac-dependent code follows documented API patterns but is **untested on a
live Isaac Sim** — expect to fix import namespaces / asset paths on first run and record them in
`experiments/ENV.md`.

## License & safety

Code: Apache-2.0 (planned). **NVIDIA USD/SimReady assets are not re-hosted** — downloaded at install and
referenced. This is **simulation research**; any physical cutting system is safety-critical and outside
the scope/warranty of these tools.
