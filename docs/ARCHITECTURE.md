# Architecture & Decision Records — project "roba"

Binding technical decisions, each grounded in the verified research (`FEASIBILITY.md`,
`REFERENCES.md`). ADRs are numbered; change them via a new ADR that supersedes, don't edit in place.

---

## System overview

```
                          ┌──────────────────────────────────────────────┐
                          │                Isaac Sim 6.0 (PhysX)          │
                          │                                               │
  mouse (x,y) ──► teleop ─┼─► plane-constrained IK (Lula) ──► Cutting arm │
                          │                                  (OpenArm A)  │
   omni.ui panel ────────►│   OSC / impedance hold ─────────► Holding arm │
   (params, start/stop)   │                                  (OpenArm B)  │
                          │                                               │
                          │   pork-belly model: 3 layers (skin/fat/lean)  │
                          │   CUTTING ── pre-scored breakable seams ◄──────── interactive demo
                          └───────────────────┬───────────────────────────┘
                                              │  (offline / alongside)
                                              ▼
   tutorial / surgical video ──► perception ──► trajectory + strategy prior
                                              │
   real cut + F/T sensor ──► DiSECt/MPM ──► calibrated force + cutting model  ◄── physics-faithful
```

Two **decoupled** cutting representations on purpose: a cheap, robust one for the live mouse demo and
a physics-faithful one for the science. They share the arm/scene/material parameterization.

---

## ADR-001 — Cutting simulation strategy (THE pivotal decision)

**Context.** Isaac Sim / PhysX cannot cut deformable soft bodies at runtime (no tearing/fracture API;
NVIDIA "not supported"; feature request #258 closed). The flashy demos are breakable-joint fakes along
predetermined seams. See `FEASIBILITY.md` Pillar 3.

**Decision.** Use **two representations**, selected by purpose:

- **Interactive / UI / teleop demo (Aim 1.4–1.5): pre-scored breakable seams (option b).**
  Author the pork belly as layered sub-bodies joined by breakable `PhysicsJoint`s along a candidate cut
  grid. The mouse-driven blade snaps joints whose break-force threshold is exceeded, with the threshold
  per-layer-calibrated to the fracture-toughness priors (skin ≫ fat > lean). Robust, real-time, runs in
  Isaac Sim today. **Known limitation (document it honestly):** cuts follow predetermined seams, not
  arbitrary tool paths. A denser seam grid + visual mesh-swap (option d) hides this for the demo.

- **Physics-faithful cutting model + force calibration (Aim 2.3–2.4, 3): DiSECt (option c1).**
  NVIDIA/USC's **differentiable** FEM cutting simulator is purpose-built for knife-through-tissue and
  exposes gradients for **parameter inference** — i.e. fit material/fracture/friction params to real F/T
  sensor data. Run it **alongside** Isaac Sim (not inside PhysX). MPM (Warp MPM / CRESSim-MPM) is the
  fallback if DiSECt's FEM is too slow/unstable for thin-layer skin skiving.

**Rejected.** (a) custom in-PhysX re-tetrahedralization — research-grade, no API/sample, unlikely
real-time, breaks across PhysX 5.x; verified high-risk. Pure (d) visual fake — fine for eye-candy, no
physics value for Aims 2/3.

**Consequence / required de-risking.** Build a **throwaway spike** for each path before committing the
timeline: (1) breakable-seam blade cut in Isaac Sim; (2) DiSECt cut of a 3-layer block calibrated to one
real force-vs-depth profile. Treat real-time topological cutting as out of scope.

---

## ADR-002 — Robot arms

**Context.** Spec names "reBot Arm x2 and OpenArm02". Both real; only OpenArm ships a validated USD and a
**bimanual** asset; reBot B601 has no Isaac USD yet (target ~2026-06-20, slip-prone) and only ~1.5 kg payload.

**Decision (resolved 2026-06-10).** **Two distinct arms**, per the user's choice:
- **Cutting arm = OpenArm (Enactic), single-arm USD** (`openarm_unimanual.usd`). 7-DOF, ~6 kg peak
  payload, backdrivable QDD motors **with force feedback** = best fit for the force-controlled cut. Pin a
  commit for the USD (assets are a "temporary" external-repo solution).
- **Holding arm = reBot Arm B601** (self-converted URDF→USD from `reBotArmController_ROS2`). Satisfies the
  spec's "import two arm models" requirement.

**Caveat — reBot payload.** B601 payload is only ~1.5 kg, marginal against cutting reaction forces. In
sim this is sidestepped: once the holding arm grasps, **weld the workpiece with a D6/Fixed joint or
surface gripper** (ADR-004) so the arm carries kinematic position, not the full cut reaction. If reBot
proves inadequate even so, fall back to OpenArm's `openarm_bimanual.usd` for both arms.

**Consequence.** Replace the shipped grippers with a custom **knife/blade end-effector** (neither arm
ships a blade). Author its collision geometry, mass, and the blade-edge contact model. Validate URDF
inertials/collision for sim-grade contact (URDF visual STL ≠ sim collision mesh).

---

## ADR-003 — Physics backend: PhysX, not Newton

**Decision.** Use **PhysX 5** for all dual-arm/contact work. **Reason (verified):** Newton's Isaac Sim 6.0
integration is experimental; its MuJoCo-Warp solver pre-allocates a fixed `nconmax`=200 contact buffer and
**silently drops** excess contacts (two arms gripping a deformable workpiece is exactly that regime);
deformable/surface-gripper/material-randomization features are PhysX-only; no dual-arm robots in Newton's
test set. Re-evaluate Newton only after its Isaac Lab integration reaches GA with a published *fidelity*
(not just speed) validation.

---

## ADR-004 — Control stack

- **Holding arm:** low-stiffness/effort grip → then "weld" the workpiece with a programmatic Fixed/D6
  joint or **surface gripper** once grasped (most robust). Optional Cartesian impedance via Isaac Lab OSC
  if compliant holding is needed.
- **Cutting arm (interactive):** plane-constrained **Lula IK** with (1) low-pass mouse target, (2)
  IK-success-flag gating with hold-last-good, (3) previous-solution seeding.
- **Cutting arm (autonomous task):** **RMPflow or cuRobo** trajectory + **hybrid force/velocity
  (FDCC-style)** control. Force role = *bound* interaction force + add slicing (slice-push) motion, **not**
  track a fixed normal setpoint (verified: the neat "force-on-normal" split is an idealization).
- **Force feedback caveat:** OSC closed-loop force is 3 linear axes only; moment feedback is open-loop.
  If slicing-moment feedback is essential, build a custom estimator from `body_incoming_joint_wrench_b`
  (wrist fixed-joint) and budget time to filter/validate it. Validate FEM-vs-rigid contact sensing early
  (Issue #4290).

---

## ADR-005 — Perception & force pipeline (decoupled)

**Decision.** Two separate stages, never conflated:

1. **Video → kinematics/strategy** (RGB tutorial/surgical footage): tool/hand pose, optical/scene flow,
   phase/action recognition, VLM captioning → cut paths, approach angles, slice-vs-press patterns,
   holding/fixturing points. This is the *trajectory/skill prior*.
2. **Sensor + sim → force/cutting model**: instrument real cuts with a wrist/blade **F/T sensor** →
   force-vs-depth/velocity ground truth → calibrate DiSECt/MPM material params (differentiable param ID).

**Reason (verified).** Quantitative forces are not recoverable from RGB (ill-posed; blade occluded
inside tissue). Vision-only force "textures" lose physical units — qualitative cue only.

---

## ADR-006 — UI

**Decision.** In-app **`omni.ui` + `isaacsim.gui.components`** custom Kit extension: sliders/checkboxes for
sim params (layer stiffness/toughness, blade sharpness, slice-push ratio, grip force), buttons wired to
World `play/pause/stop/reset`. Mouse teleop polled from `carb.input` in the step loop. Defer any external
(web/PyQt) panel unless headless/remote operation is required (then ROS 2 bridge or a small socket sidecar).

---

## ADR-007 — Repo, reproducibility, licensing

**Decision.** Layout mirrors **IsaacLabExtensionTemplate**. Pin Isaac Lab↔Isaac Sim↔Python↔CUDA/PyTorch↔
NGC Docker digest. Hydra configs + experiment tracker + multi-RNG seeding. **Do not vendor NVIDIA USD/
SimReady assets** — ship a setup script that downloads them at install and *references* them; ship only
self-authored USD + Apache-2.0 code. Verify each 3rd-party (reBot/OpenArm/knife) asset license before
redistribution. arXiv **cs.RO**. Carry an explicit **safety disclaimer** (NVIDIA license disclaims
liability for injury-capable systems; this is sim-only research).

---

## ADR-008 — Compute platform: BU SCC (A40/A6000), Mac as thin client

**Context.** No local RTX workstation; candidate hosts are the user's **Mac** and the **BU Shared
Computing Cluster (SCC)**. Verified facts: (1) Isaac Sim has **no macOS support** (Ubuntu 22.04/24.04 or
Win 10/11 + NVIDIA RTX only) — the Mac cannot host the sim at all. (2) Isaac Sim needs **RT cores +
NVENC**; SCC's **A100/A100-80G/H200 are unsupported** (no RT cores), but SCC also has **A40 (~48 free),
A6000 (~50 free), and L40S** — all with RT cores → supported.

**Decision.** **Run on SCC requesting `gpu_type=A40` (or A6000).** Prefer A40/A6000 over L40S (one forum
report of L40S detection trouble). The **Mac is a thin client** (browser/VNC/SSH). Workflow:
- **Container:** build the NGC Isaac Sim image with **Apptainer/Singularity** (no Docker on SCC — users
  aren't root), run with `--nv`.
- **Interactive mouse demo:** **SCC OnDemand GPU desktop (VNC)** or **WebRTC livestream** to the Mac
  browser. Accept some input latency (fine for slow cutting; not twitchy).
- **Headless/batch** (RL, DiSECt calibration, rendering): native on SCC.
- **Verify with RCS:** the A40-node **NVIDIA driver version**. Isaac Sim 6.0 wants ≥ 580.65.06; if older,
  **pin to Isaac Sim 5.1** instead.

**Consequence.** Phase 0.1 includes Apptainer build + an OnDemand/WebRTC connectivity test before any
sim work. A local Linux/Windows RTX box, if ever available, would give smoother teleop but is optional.

---

## Resolved planning decisions (2026-06-10)

1. **Hardware:** ✅ BU SCC A40/A6000 (ADR-008); Mac cannot host Isaac Sim.
2. **Demo cutting fidelity:** ✅ **breakable seams** for the interactive demo (ADR-001 option b);
   DiSECt reserved for the physics-faithful Aim 2/3 model.
3. **F/T sensor:** ⏳ **planning to acquire** — design the force pipeline (ADR-005) to accept real data
   later; until then calibrate DiSECt to synthetic/literature data, clearly caveated as sim-only.
4. **Arms:** ✅ **two distinct arms** — **OpenArm (cutting)** + **reBot B601 (holding)** (ADR-002);
   reBot needs URDF→USD self-conversion.
