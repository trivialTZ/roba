# Roadmap — project "roba"

Phased plan, re-scoped per `FEASIBILITY.md` and `ARCHITECTURE.md`. Each phase ends with a concrete,
demonstrable artifact and an explicit go/no-go. Phase 0 exists specifically to retire the two big risks
**before** investing in the full build.

Legend: 🔴 high-risk / de-risk first · 🟡 medium · 🟢 low.

---

## Phase 0 — De-risking spikes (do this first) 🔴

> Goal: prove the two load-bearing unknowns are tractable on *your* hardware before committing.

- [ ] **0.1 Compute + install (BU SCC).** Request an interactive **A40** (or A6000) GPU session
      (`qrsh -l gpus=1 -l gpu_type=A40 ...` or via SCC OnDemand). Build the **NGC Isaac Sim image with
      Apptainer/Singularity**, run with `--nv`. Confirm the node **driver ≥ 580.65.06** (else pin Isaac Sim
      **5.1**). Test the **OnDemand VNC / WebRTC livestream** path to the Mac. Record exact versions in
      `experiments/ENV.md`. *(Mac cannot host the sim — client only. ADR-008.)* Use the ready-made kit in
      `deploy/scc/` and **follow `deploy/scc/SCC_USAGE.md`** (no login-node compute; `/projectnb` not home;
      release idle GPUs) to avoid policy flags.
- [ ] **0.2 Arm import spike.** Drop **OpenArm bimanual USD** into an empty scene; move both arms with
      joint targets. Self-convert **reBot B601 URDF→USD** and verify it articulates. (ADR-002)
- [x] **0.3 ✅ Cutting spike A (interactive path) — DONE 2026-06-10.** Pork-belly as 20×3 layered
      sub-blocks + 97 breakable seams; scripted blade sweep cuts seams top-down in a physically sensible
      per-layer pattern (skin/fat/lean X-seams + interlayer peel seams). Validated **headless in real PhysX
      on SCC** (job 6027631, 49 s). Metrics in `experiments/out/headless_cut_results.json`; figure via
      `experiments/plot_cut.py`. (ADR-001 option b)
- [ ] **0.4 🔴 Cutting spike B (physics path).** Stand up **DiSECt**; cut a 3-layer block; fit one material
      param to a single force-vs-depth curve (synthetic if no sensor yet). **Go/no-go:** does DiSECt run and
      calibrate? If too slow/unstable for thin skin → evaluate Warp MPM. (ADR-001 option c1)
- [ ] **0.5 Decision gate.** Confirm ADR-001 split or revise. Write up findings; update `ARCHITECTURE.md`.

**Exit criterion:** both cutting paths demonstrated at toy scale; hardware confirmed; arms import cleanly.

---

## Visualization (driver-agnostic, unblocks the 595 RTX gap) ✅ DONE 2026-06-10

The SCC 595 driver blocks Isaac's RTX renderer, but headless physics works. So we **record per-frame
box poses** during a headless run (`src/roba_sim/recording.py`) and **replay them with matplotlib 3D**
(`experiments/render_replay.py`) — no NVIDIA driver / Isaac renderer needed. Validated: recorded the
slice cut (80 frames × 61 boxes) on SCC, rendered a GIF + stills on the Mac showing the layered slab
(red lean / cream fat+skin) + knife. This fully unblocks visualization without RCS; turns the
interactive mouse demo into record→replay→render.
- Polish item: freed sub-blocks get a large impulse from the kinematic blade (penetration) and can fly
  off — tune blade speed / contact, or break seams slightly ahead of contact, for cleaner clips.

## Deformable cutting — "feels like meat" (DONE 2026-06-10) ✅

The rigid breakable-seam model can't deform (rigid bricks). To make the meat actually squish + part,
we built a **mass-spring soft body in NVIDIA Warp** (`experiments/warp_cut.py`) — the deformable
upgrade of the breakable-seam idea: lattice nodes deform under the knife; springs break when the blade
passes or strain exceeds a per-layer threshold (skin>lean>fat ordering). Runs on **CUDA via warp-lang**
(a `/projectnb/pi-brout/$USER/roba_work/wenv` venv), so the 595 RTX-driver block is irrelevant — it's
compute. 3600 nodes / 28.6k springs, stable (explicit, dt 3e-5 × 60 substeps, velocity-clamped),
4182 springs cut at 3 stations. Rendered driver-free with VTK as a solid layered slab that visibly
deforms and parts (`experiments/render_softbody.py` → `out/warp_cut.gif`, stills `warp_cut_f{0,28,54}.png`).
This realizes ADR-001 option (c) — the physics-faithful path — for visualization. (Stiffness is
visual-scaled for stable real-time; absolute cutting forces stay in the force_cut/breakable-seam results.)
Next refinement: calibrate spring stiffness/break to real Pa/toughness; add the robot knife driving it.

## Phase 1 — Aim 1: interactive dual-arm cutting sim 🟡

> Goal: deliver the spec's Aim 1 end-to-end — mouse-driven cutting with a holding arm and a UI.

- [ ] **1.1 Scene & material (Aim 1.2).** Build the 3-layer pork belly (`MATERIAL_MODEL.md` params,
      domain-randomized). Wire layer stiffness/toughness/friction to the breakable-seam thresholds.
- [x] **1.2 Dual-arm setup (Aim 1.3) — DONE 2026-06-10.** OpenArm **bimanual USD (22 DOF = 2×7-DOF)**,
      knife on the right TCP, 3-layer slab. **IK-driven robot cut validated headless** (`experiments/
      dual_arm_ik_cut.py`, job 6035210): finite-difference-Jacobian damped-least-squares IK (no
      URDF/Lula needed) raises the EE to a cut-ready pose then descends the knife through the slab,
      breaking 10 seams (fraction 0.103) exactly as the blade crosses the layers. Figure `out/ik_cut.png`.
      Key findings: use `ic.Articulation` (the batched core.prims view didn't actuate); home pose is at the
      bottom of the workspace so cut = raise-then-descend; reBot URDF-import API fixed for Isaac 6.0.
      NEXT (polish): proper holding-arm grasp+weld on the left arm; press-push-slice lateral motion.
- [ ] **1.3 Plane-constrained IK + mouse teleop (Aim 1.4).** Lula IK constrained to the cutting plane;
      `carb.input` mouse → in-plane (X,Y); low-pass + IK-success gating + previous-seed. (ADR-004, Pillar 6)
- [ ] **1.4 UI (Aim 1.5).** `omni.ui` + `isaacsim.gui.components` panel: param sliders (layer props, blade
      sharpness, slice-push ratio, grip force), start/stop/reset buttons → World. (ADR-006)
- [ ] **1.5 Integration demo.** Move mouse → cutting arm slices the held pork belly along the seam grid;
      params adjustable live. Record a screen capture for the paper.

**Exit criterion:** a person drives the cut with the mouse; the holding arm stabilizes; UI controls work.

---

## Phase 2 — Aim 2: data-driven control + modeling 🟡

> Goal: the literature-grounded control models + the (re-scoped) video→model pipeline.

- [ ] **2.1 Control-theory writeup (Aim 2.1).** Synthesize Jia mechanics / slice-push, DiSECt, STAR/SRT-H,
      force/impedance/visual-servoing roles into `docs/CONTROL_SURVEY.md` (skeleton from `REFERENCES.md`).
- [x] **2.2 Perception: video → trajectory (Aim 2.3, stage 1) — EXECUTED 2026-06-10.** `perception/
      video2traj/` ran end-to-end on a synthetic press-push-slice clip: recovered 75 samples, the 3-station
      structure, the slicing oscillation, phase labels (approach/press/slice), slice-push estimate 0.38;
      mean normalized tracking error 0.16 vs ground truth (centroid-vs-tip bias). `perception/
      make_synthetic_cut_video.py`, `validate_traj.py`; figure `out/traj_validation.png`. (Real-dataset
      footage — EPIC-KITCHENS/Cholec80 — is the next step; forces remain sensor/sim, ADR-005.)
- [ ] **2.3 Force model: sensor/sim → forces (Aim 2.3, stage 2).** Calibrate DiSECt material params to F/T
      data (real if available, else documented synthetic). Decoupled from 2.2. (ADR-005)
- [x] **2.4 Task model (a): skin skiving (Aim 2.4a) — VALIDATED headless 2026-06-10.** 🔴 novel.
      Constant-depth pass selectively severs **20/20 skin/fat interfaces (peel fraction 1.0)** while leaving
      lean/fat and skin column-seams intact → skin peels as a connected sheet. `experiments/skin_skive.py`,
      results in `out/skin_skive_results.json`, figure `out/skive_selectivity.png`. (Next: real impedance +
      surface-following control + real pork-rind validation.)
- [x] **2.5 Task model (b): vertical slicing (Aim 2.4b) — VALIDATED headless 2026-06-10.** 3-station blade
      sweep, monotonic cut progression (`experiments/headless_cut.py`). (Next: press-push-slice with the
      ~40% slicing force reduction quantified against the force model.)
- [x] **Force-model validation (Aim 1.2/2.2) — DONE 2026-06-10.** PhysX dynamically honors the per-layer
      toughness→break-force model: measured sever forces match material.py (fat 2.3%, skin 0.5%; lean within
      one ramp step), reproducing skin≫fat>lean. `experiments/force_cut.py`, figure `out/force_break.png`.

**Exit criterion:** two task controllers run in sim; video pipeline yields trajectory priors; force model
calibrated and caveated.

---

## Phase 3 — Aim 3: integration, docs, paper 🟢

- [ ] **3.1 Merge** Aim-1 sim with Aim-2 control/modeling (shared scene/material params).
- [ ] **3.2 Experiments** with tracked configs (Hydra + W&B), seeds, and metrics. (ADR-007)
- [x] **3.3 GitHub repo — DONE 2026-06-10.** Public at **https://github.com/trivialTZ/roba** — Apache-2.0,
      72 files, figures + PDF, no NVIDIA assets re-hosted (`.gitignore` excludes `assets/robots/`), pinned
      env in `experiments/ENV.md`, safety disclaimer in README. (ADR-007)
- [x] **3.4 PDF paper — DONE 2026-06-10.** `paper/main.tex` → `paper/main.pdf` (IEEE 2-column, compiles
      clean, 0 TODOs): system, methods, 4 result figures, full limitations section.

**Exit criterion:** reproducible repo + paper; honest limitations section.

---

## Risk register (live)

| Risk | Likelihood | Impact | Mitigation | Owner phase |
|------|-----------|--------|------------|-------------|
| Native cutting unavailable | **certain** | high | ADR-001 dual representation; Phase-0 spikes | 0 |
| ~~No RTX GPU~~ → resolved: SCC A40/A6000/L40S | — | — | use SCC (ADR-008); Mac is client-only | 0 |
| **RTX rendering broken on SCC driver 595.71.05** (IsaacSim#537) | **confirmed** | **high (viz only)** | headless physics WORKS (workaround in job_headless.qsub); ask BU RCS for a ~580-driver node for the interactive GUI demo | 0/1 |
| ~~driver < 580.65.06~~ → driver is 595.71.05 (too NEW, regression) | — | — | n/a — rebuilding 5.1 won't help (#537 affects both) | 0 |
| VNC/WebRTC teleop needs RTX (currently blocked) | high | med | gated on the RCS driver fix; develop physics headless meanwhile | 1 |
| DiSECt too slow for thin skin | medium | med | fall back to Warp MPM | 0/2 |
| No real F/T sensor | medium | med | sim-only force model, caveat clearly | 2 |
| reBot USD slips past 2026-06-20 | medium | low | self-convert URDF (already planned) | 0 |
| Muscle anisotropy/param scatter | high | med | domain randomization; local characterization | 1/2 |
| Skin-skiving has no prior baseline | high | med | frame as contribution; real-skin validation | 2 |
| Sim-to-real gap (if hardware) | high | high | RL + domain randomization; out of scope if sim-only | 3 |

---

## Suggested sequencing notes

- **Do Phase 0 before promising Aim-1 dates.** The cutting spikes are where schedule risk actually lives.
- Phases 1 and 2.1–2.2 can overlap (sim build vs. literature/perception are independent).
- The **paper's strongest novelty** is the skin-skiving control model (2.4a) + the honest Isaac-Sim
  cutting-workaround analysis — lead with those.
