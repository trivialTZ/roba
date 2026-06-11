# Feasibility Assessment — Robotic Meat-Cutting Simulation (project "roba")

**Date:** 2026-06-10 · **Status:** pre-implementation briefing · **Method:** 12 parallel
research agents + 17 adversarial verification agents over 152 sources (see `REFERENCES.md`).

This document records what is **true today** about each technical pillar of the project,
with special emphasis on the assumptions in the original spec that do **not** hold and must
be re-scoped before any code is written.

---

## TL;DR — the three things you must internalize before starting

1. **Isaac Sim / PhysX cannot cut a deformable soft body out of the box.** There is no
   tearing/fracture/re-meshing API for FEM soft bodies, and NVIDIA staff state arbitrary
   cutting is "not supported." The viral "cutting in Isaac Sim" demos are *fakes*:
   pre-segmented pieces glued by breakable joints that snap along **predetermined seams**.
   Genuine tool-driven severing of a continuum is an open R&D problem. **This is the
   single largest risk in the project and it touches Aims 1.2, 1.3, 2.3, 2.4, and 3.**
   → See *Pillar 3* and the Architecture decision in `ARCHITECTURE.md`.

2. **You cannot read cutting *forces* off ordinary video.** Trajectories/kinematics from
   RGB tutorial/surgical footage: realistic. Quantitative, unit-bearing contact forces from
   RGB: not possible (ill-posed, and the blade is occluded *inside* the tissue mid-cut).
   Aim 2.3 must be re-scoped to "video → kinematic/strategy model" + a **separately
   sensor-/sim-calibrated force model." → See *Pillar 10*.

3. **Hardware gate.** Isaac Sim requires a dedicated RTX GPU with RT cores (min RTX 4080 /
   16 GB VRAM, 32 GB RAM). Data-center A100/H100 are explicitly **unsupported**. Confirm the
   target machine meets this before committing the Isaac Sim plan. → See *Pillar 1*.

Everything else in the spec is feasible, most of it as stated.

---

## Pillar-by-pillar verdicts

| # | Pillar | Spec Aim | Verdict | One-line reason |
|---|--------|----------|---------|-----------------|
| 1 | Isaac Sim platform | 1.1 | ✅ feasible-as-stated | Mature; current GA = **Isaac Sim 6.0** (2026-06-08); pin a version. Needs RTX GPU. |
| 2 | Named robot arms | 1.1 | ✅ feasible (with nuance) | Both arms **real**; OpenArm ships a USD, reBot needs self-import. |
| 3 | **Cutting deformables** | 1.2/1.3 | 🔴 **hard / major risk** | **No native cutting in Isaac Sim.** Re-architect (see below). |
| 4 | Pork material params | 1.2 | 🟡 partial data | Skin & fat measured; muscle poorly measured + anisotropic; **calibrate locally.** |
| 5 | Dual-arm impedance/hold | 1.3 | 🟡 feasible-with-changes | PD spring-damper + Isaac Lab OSC; force control limited to 3 linear axes. |
| 6 | IK → 2D → mouse | 1.4 | ✅ feasible-as-stated | Lula/cuRobo IK; constrain to cutting plane, map mouse XY → in-plane XY. |
| 7 | Simulation UI | 1.5 | ✅ feasible-as-stated | `omni.ui` + `isaacsim.gui.components`; World play/pause/stop. |
| 8 | Cutting control theory | 2.1 | ✅ rich literature | Jia mechanics + slice-push ratio; DiSECt; STAR/SRT-H surgical. |
| 9 | Force/impedance/visual servoing | 2.2 | ✅ rich literature | Vision plans path; force/impedance governs contact; they layer. |
| 10 | **Video → physical model** | 2.3 | 🟡 feasible-with-changes | **Trajectories yes, forces no** from RGB. Decouple the pipeline. |
| 11 | Two cutting tasks | 2.4 | 🟡 feasible-with-changes | Slicing maps to Jia press-push-slice; **skin-skiving is a novel gap**. |
| 12 | Repro + paper | 3 | ✅ best-practices known | IsaacLabExtensionTemplate; cs.RO; don't re-host NVIDIA assets. |

---

## Pillar 1 — NVIDIA Isaac Sim (Aim 1.1) ✅

- **Current GA is Isaac Sim 6.0 (released 2026-06-08).** Prior stable line: 5.0 (2025-08) /
  5.1. Cadence ≈ one major release every 6–12 months — **pin your version** for reproducibility.
- Hierarchy: **Omniverse** (foundation) → **Isaac Sim** (reference app) → **Isaac Lab**
  (GPU robot-learning framework). Apache-2.0, free for development.
- Physics backend: **PhysX 5** (mature). 6.0 adds *experimental* **Newton** (MuJoCo-Warp solver).
  **Verified recommendation: stay on PhysX.** Newton's Isaac Sim integration is experimental,
  has a fixed contact buffer (`nconmax`=200) that *silently drops* excess contacts — exactly the
  failure mode for a two-arm-grips-deformable scenario — and no dual-arm robots are in its test set.
- Robots imported via the open-sourced **URDF/MJCF importer** → USD, with "Fix Base Link".
- Two workflows: **standalone `SimulationApp`** (CLI, headless or GUI, scriptable) and the
  interactive **GUI/script editor**.
- **Hardware (gating):** min RTX 4080 / 16 GB VRAM, 32 GB RAM, Ubuntu 22.04/24.04 or Win 10/11;
  recommended RTX 5080-class. **A100/H100 unsupported (no RT cores).** Omniverse Launcher was
  deprecated 2025-10-01 — install via pip/Docker/Pixi; older tutorials may be stale.
- **This project's compute (resolved, ADR-008):** the user's **Mac cannot host Isaac Sim** (no macOS
  build). **BU SCC** is the host — request **A40 or A6000** nodes (RT cores + NVENC → supported); **do not
  request A100/A100-80G/H200** (no RT cores → unsupported). Run via **Apptainer/Singularity** (no Docker on
  SCC); interactive demo via **SCC OnDemand VNC** or **WebRTC livestream** to the Mac; verify the A40-node
  driver is ≥ 580.65.06 or pin **Isaac Sim 5.1**.

## Pillar 2 — The two named arms (Aim 1.1) ✅ with nuance

**Both arms are real.** The spec's alias lists conflate distinct products — use these exact identities:

- **reBot Arm = "reBot Arm B601"** (variants B601-DM / B601-RS; code-named *reBot-DevArm*), by
  **Seeed Studio**, launched 2026-04. Fully open (CAD/STEP, ROS2, Pinocchio, LeRobot).
  - **Caveat:** the importable **URDF lives in the ROS2 *controller* repo**
    (`Seeed-Projects/reBotArmController_ROS2`), *not* the headline hardware repo. **No official
    Isaac Sim USD yet** — marked "in progress," target ~2026-06-20 (slip-prone). You must run
    URDF→USD yourself and tune physics. **Payload only ~1.5 kg**, ships a parallel gripper (no blade).
  - The spec aliases "ReBeL/ReByte/RB arm" are **wrong** — *ReBeL* is an unrelated igus plastic cobot.
- **OpenArm02 = "OpenArm" (v2 era)** by **Enactic, Inc.** (Tokyo). The "02" is a version marker
  (`openarmv2.urdf`; hardware release "OpenArm 01: Release No.2"), not a separate product.
  - **Ships both URDF *and* a USD** (`enactic/openarm_isaac_lab`: `openarm_unimanual.usd`,
    `openarm_bimanual.usd`), 7-DOF, ~633 mm reach, **~6.0 kg peak payload**, high-backdrivability
    QDD motors with force feedback, ROS2/MoveIt2/LeRobot. **Watch for a name collision** with an
    unrelated `wanweiwei07/OpenArm` — use the **Enactic** repos.

**Implication:** OpenArm is the stronger contact-rich-cutting platform and the *only* one shipping a
validated Isaac USD + a **bimanual** asset. Recommendation in `ARCHITECTURE.md` (ADR-002).

## Pillar 3 — Cutting deformables in Isaac Sim (Aim 1.2/1.3) 🔴 THE central risk

**Verified REFUTED (high confidence):** Isaac Sim / PhysX 5 cannot cut/separate a deformable
mesh at runtime.

- PhysX FEM soft bodies use a co-rotational FEM on a **fixed tetrahedral mesh**. Docs only note
  that precomputed data *"must be updated if topology is changed by the application"* — but **no
  API exists** to perform the cut, re-tetrahedralize, or rebuild remap tables. Cooking is an
  offline CPU op, not per-frame.
- NVIDIA staff (2025-06): *"cutting deformables at arbitrary points is not supported in IsaacSim."*
  Feature request #258 (2025-10) closed with no native solution. Latest PhysX 5.6 changelog: no
  cutting/tearing/fracture feature. Legacy "Deformable Body" was **deprecated in 5.0**; new
  "Deformable Bodies (Beta)" still requires fixed topology.
- The "cucumber/sausage cutting" demos are **breakable-joint fakes** (e.g. 70 N break-force on
  pre-segmented pieces). The blade snaps predefined joints; nothing is topologically cut.

**Realistic options (all require real engineering — see ADR-001):**

| Option | What it is | Pro | Con |
|--------|-----------|-----|-----|
| **(b) Pre-scored breakable seams** | Author meat as sub-bodies joined by breakable joints, broken along a scripted seam | Cheap, robust, runs in Isaac Sim today | Cuts follow **predetermined** seams, not arbitrary tool paths |
| **(c1) DiSECt** | NVIDIA/USC **differentiable** FEM cutting simulator, purpose-built, calibratable to force data | Physically faithful; gradients for parameter ID; built for *exactly this* | Separate engine; integrate alongside Isaac Sim, not inside PhysX |
| **(c2) MPM** (Warp MPM / CRESSim-MPM / TopoCut) | Material Point Method; fractures without remeshing | Severing & big topology change natural | Own fidelity/contact/integration cost; not a turnkey cutting solver |
| **(a) Custom PhysX re-tet** | Re-tetrahedralize + re-cook live inside PhysX | Stays in-engine | Research-grade, fragile, no API/sample, unlikely real-time |
| **(d) Visual fake** | Deform, then swap to a pre-cut mesh / spawn pieces on trigger | Trivial, great for UI demo | Not physically faithful |

**Recommended split:** use **(b) pre-scored seams** for the interactive mouse demo (Aim 1.4) and
**(c1) DiSECt** for the physics-faithful cutting model and force calibration (Aims 2/3). Prototype
the chosen path in a throwaway spike **before** committing the timeline.

## Pillar 4 — Pork-belly material model (Aim 1.2) 🟡 partial

Cutting physics is governed by **fracture mechanics** (energy-per-area, J/m²) + blade sharpness +
friction, *not* simple strength. Verified numbers (order-of-magnitude priors; **calibrate locally**):

| Layer | Fracture toughness Jc | Stiffness | Notes |
|-------|----------------------|-----------|-------|
| **Skin (dermis)** | **~17–30 kJ/m²** (toughest, dominates initial cut) | MPa-scale in tension, hyperelastic | Best data is *juvenile* piglet skin (Comley & Fleck 2010 ~17; Pissarenko 2020 ~20–30) |
| **Subcutaneous fat** | **~4.1 kJ/m²** (measured! Comley & Fleck 2010, trouser-tear) | shear modulus ~1–5 kPa, varies 30–60× by depot | The initial "no fat data" worry was **refuted** — a real value exists |
| **Lean muscle** | **~0.1–0.84 kJ/m²** but ~100% scatter; many soft-tissue measurements judged *invalid* (Taylor 2012) | tens–hundreds kPa, **strongly anisotropic** (fiber direction) | A single scalar cannot capture it; grain & temperature dependent |

- **Slice/draw motion** (the "slice-push ratio") cuts required force by **tens of % (~40%)** vs pure
  pressing — a first-class control variable, not a detail.
- **Blade-tissue friction coefficient is the real data gap** (cutting force is ~30–45% friction for
  bacon). Treat μ as a tunable parameter; run a sensitivity sweep or a quick benchtop tribology test.
- **Action:** use literature as priors with **domain randomization** over the ranges; budget for
  in-house material characterization (trouser/wedge tests + instrumented blade). See `MATERIAL_MODEL.md`.

## Pillar 5 — Dual-arm hold + impedance/damping control (Aim 1.3) 🟡

- Every PhysX joint is an implicit **PD spring-damper**: `τ = k(q−q*) + d(q̇−q̇*)`, clamped by max
  force. High `k` = stiff tracking; low/zero `k` + `d` = compliant; `k=0` + `d` + effort target =
  **force/torque control** → suitable for compliant grasping.
- **Isaac Lab ships an Operational Space Controller (OSC):** Cartesian impedance, hybrid force-motion,
  inertial decoupling, gravity comp, variable impedance, null-space control. Compute-only — you feed
  Jacobian, mass matrix, poses/velocities, optional contact force.
- **Verified limitation:** OSC closed-loop **force control is effectively 3 linear axes** (via contact
  sensors). A true 6-DoF *contact* wrench is not cleanly available; a joint **incoming-wrench** API
  exists (`body_incoming_joint_wrench_b`) but is reaction-at-joint, noisy, with open accuracy
  complaints. **Moment (slicing-shear) feedback is open-loop** out of the box.
- **Hold-and-manipulate recipe** (no turnkey task exists — assemble from primitives): Arm A grasps and
  holds via low-stiffness/effort grip *or* a programmatic Fixed/D6 joint / **surface gripper** to
  "weld" the workpiece once grasped; Arm B runs OSC or a **cuRobo**-planned trajectory. cuRobo can plan
  both arms but treats them as one robot (multi-arm is experimental).
- **Deformable-contact gap:** ContactSensor support for rigid-vs-FEM contact is unconfirmed (open
  Issue #4290). For the holding arm gripping deformable meat, validate contact sensing early.

## Pillar 6 — IK → 2D plane → mouse (Aim 1.4) ✅

- **IK options:** **Lula** (`LulaKinematicsSolver`, backs RMPflow) and **cuRobo/cuMotion** (GPU, 20–80×,
  collision-aware) are first-class in Isaac Sim. IKFast / PyKDL / Pinocchio(`pink`) are bring-your-own.
- **Pattern:** don't feed 2D pixels to a full 6-DoF IK. **Reduce to the cutting plane** — fix EE
  orientation + one Cartesian axis (e.g. constant Z, downward tool), map the **two mouse variables →
  the two in-plane coords (X,Y)**. Only 2 of 6 task-space DOF are commanded.
- **Mouse capture:** poll `carb.input` (`acquire_input_interface` +
  `get_mouse_coords_pixel/normalized`) inside the update loop (no event callbacks). Isaac Lab's
  `Se3Keyboard`/`Se3SpaceMouse` device classes show the canonical `device.advance() → SE(3) →
  IK action → env.step` pipeline; a 2D mouse device drops into the same slot.
- **Smoothness:** warm-started Lula IK is likely smooth enough for the 2-DOF planar demo. Three cheap
  fixes beat premature RMPflow: (1) low-pass the mouse target, (2) **always check the IK success flag**
  and hold last-good joints on failure (kills the biggest jump source near singularities), (3) seed with
  previous solution. For the **real cutting task** (collision avoidance, joint limits, optimal blade
  paths, force) use **RMPflow / a trajectory planner + force control** — not raw per-frame IK.

## Pillar 7 — Simulation UI (Aim 1.5) ✅

- Lowest-friction in-app path: a custom Kit extension using **`omni.ui`** + the higher-level
  **`isaacsim.gui.components`** builders (`btn_builder`, `float_builder`,
  `combo_floatfield_slider_builder`, `state_btn_builder`, `cb_builder`). Widgets follow Model-Delegate-
  View: register `slider.model.add_value_changed_fn(cb)`, read `get_value_as_float()`, write into the
  sim on the next physics step. Start/stop/reset = World/`SimulationContext` `play/pause/stop/step`
  wired to buttons. Copy from `isaacsim.examples.ui` / the Extension Template Generator.
- External panels (PyQt, Streamlit, web) are viable but need a transport (**ROS 2 bridge** or a small
  socket/HTTP loop polled in the sim step). NVIDIA's shipped web path is **WebRTC livestreaming**
  (remote viewing) + a documented but DIY `AppStreamer.sendMessage` channel — fine for low-rate UI
  actions, **not** a real-time control bus. **Recommendation:** `omni.ui` in-app for the demo.

## Pillar 8 — Cutting control theory (Aim 2.1) ✅ rich literature

- **Food/deformable cutting:** physics-based mechanics — fracture-mechanics + FEM force prediction and
  the **slice-push (pressing-slicing) ratio** (Yan-Bin Jia, Iowa State = leading mechanics group);
  **DiSECt** (NVIDIA/USC) = leading differentiable cutting sim. Learning-based: LfD with DMPs + force
  profiles, DiffSkill, RL + tactile/visual (Sashimi-Bot 2025).
- **Surgical cutting/resection:** **STAR program** (Axel Krieger, JHU) is the flagship — supervised-
  autonomy suturing (2016) → autonomous laparoscopic anastomosis (2022) → **SRT-H (2025)**, a
  hierarchical *language-conditioned imitation-learning* framework that autonomously performed the
  **clip-and-cut phase of cholecystectomy** (cystic duct & artery) on **n=8 ex-vivo porcine** gallbladders
  at 100% with no human takeover. *Caveat (verified):* trained on **~18,000 real teleop demos**, runs on
  real dVRK hardware — **not** a sim/sim-to-real result. **Transferable lesson:** a hierarchical high-level
  language/task planner over a low-level imitation policy with autonomous error recovery is what made
  long-horizon reliability work — design the meat-cutting agent the same way, not as one monolithic policy.

## Pillar 9 — Force / impedance / visual servoing in cutting (Aim 2.2) ✅

- **(a) Force / hybrid force-position** = the workhorse for contact (press force to fracture, knife-board
  force). Jia ICRA'19 cut onions/potatoes with a Barrett WAM + ATI 6-axis F/T via press-push-slice.
- **(b) Impedance / admittance** imposes spring-mass-damper compliance (cooperative bone milling;
  Mitsioni/Kragic data-driven MPC; SliceIt! FDCC on dual UR5e).
- **(c) Visual servoing** guides cut geometry (STAR: IBVS with NIR/ICG markers; Long et al.: IBVS via
  image moments for foam cutting).
- **Verified nuance — don't hard-code "vision=tangential, force=normal":** that neat split is an
  *idealization*, not a law. The most-cited soft-material work (Long et al.) deliberately does **not**
  regulate a normal-force setpoint — vision controls all 6 DOF incl. depth, and force is used to *bound*
  global deformation and add slicing motion. Real systems vary (STAR vision-only during the cut; SliceIt
  fuses everything). **Design:** a unified Cartesian impedance/admittance or hybrid force-velocity
  controller (à la FDCC) whose force role is to bound interaction forces + add slicing, not track a normal
  setpoint. RL is a proven path to tune those parameters in sim before transfer.

## Pillar 10 — Video → mathematical-physical model (Aim 2.3) 🟡 RE-SCOPE

**Verified: trajectories yes, quantitative forces no.** There is no single pipeline; it is a stack:

1. **Perception** (good today): instrument/hand pose, optical/scene flow, depth, deformation tracking.
   Monocular trajectory extraction is good enough to train imitation policies (SurgiPose ~10–15 mm error,
   60–70% task success vs 80–100% with ground-truth kinematics).
2. **Semantic understanding:** phase/action/gesture recognition, VLM captioning.
3. **Physics models** (FEM + linear-elastic fracture mechanics, or **DiSECt**) fit to observations.

**Forces are NOT observable from RGB** — ill-posed inverse problem; works only with strong priors +
a force sensor for training/validation; and for meat the blade-tissue contact is **occluded inside the
tissue**. Vision-only "force textures" (e.g. SurgeMOD) recover only the *temporal shape* of a force
signal and **lose physical units** — usable as a qualitative cue after per-setup calibration, never as a
ground-truth oracle for force thresholds or RL reward.

**Re-scoped Aim 2.3:** two decoupled stages — **video → trajectory/skill prior** (cut paths, approach
angles, slice-vs-press patterns, holding points), and **sensor + sim → force model** (instrument real
cuts with an F/T sensor; calibrate DiSECt/MPM material params to that data). Expect to need a **real F/T
sensor** at physical-execution time because mid-cut the controller cannot rely on (occluded) vision.

## Pillar 11 — The two cutting subtasks (Aim 2.4) 🟡

- **(a) Skin skiving** (shallow constant-depth peel of rubbery skin) maps onto robotic peeling
  (MORPHeus, AutoPeel) + thin-film peel mechanics: keep the edge tangent at shallow constant depth via
  **Cartesian impedance + surface/tactile following**; treat it as a crack propagating along the
  skin/flesh adhesion interface (peel angle + energy-release-rate govern force). **Verified novelty:** *no
  published closed-loop robot does constant-depth skiving of chewy animal skin* — industrial deskinning is
  fixed-geometry blade-roller machines that tend to cut into flesh. This is an **integration/validation
  gap = a genuine research contribution**, not a weekend port. Frame it as such; budget a skin material
  model (hyperelastic + cohesive-zone interface) and real-skin validation (pork rind / chicken skin).
- **(b) Vertical pork-belly slicing** maps directly to **Jia's press-push-slice** + DiSECt: press down,
  push to a target contact force, add lateral slicing (translate+rotate) that empirically lowers cutting
  force ~40%. Failure modes: slippage, tissue deformation, **jamming on fat/connective layers**,
  incomplete separation. STAR-style multi-pass increasing-depth + tangential slicing are good baselines.

## Pillar 12 — Reproducibility & paper (Aim 3) ✅

- Start from the **IsaacLabExtensionTemplate** (external-project layout: `source/<ext>/` with
  `pyproject.toml`/`setup.py`/`config/extension.toml`, `scripts/train.py`+`play.py`, tasks via
  `gym.register`).
- **Pin the whole stack:** Isaac Lab release → compatible Isaac Sim version → Python (3.11 for 5.x) →
  CUDA/PyTorch → **NGC Docker image digest**. Isaac Sim/Lab break silently across versions; watch stale
  Docker volume-cache drift. Use Hydra/OmegaConf configs + W&B/MLflow/TensorBoard + explicit multi-RNG
  seeding (accept GPU/PhysX is only *approximately* reproducible).
- **Publication:** primary arXiv category **cs.RO** (cross-list cs.LG/cs.AI/cs.CV); venues CoRL/ICRA/RSS/
  IROS (~8 pages, double-blind, externally hosted code+video).
- **Licensing (verified):** **do not re-host NVIDIA USD/SimReady assets** in a public repo (the license
  bars redistribution; "reference Nucleus/download at install time" is the community workflow the official
  IsaacSim repo uses). Your own authored USD + Apache-2.0 code are fine; check each 3rd-party robot/gripper/
  knife asset license. **Safety note:** the NVIDIA license disclaims liability for injury-capable systems
  and caps liability at $5 — a physical cutter cannot rely on NVIDIA warranties.

---

## Net feasibility statement

The project is **feasible and worthwhile**, but the original spec assumes a capability (native
deformable cutting in Isaac Sim) that does not exist and an information channel (forces from video) that
is physically unavailable. With the two re-scopes above — **(1) cutting via pre-scored seams for the
interactive demo + DiSECt/MPM for the physics-faithful model, and (2) video for kinematics + sensor/sim
for forces** — every Aim has a defensible path. The skin-skiving subtask is, pleasingly, a **novel
research contribution** rather than a re-implementation. See `ROADMAP.md` for the phased plan and
`ARCHITECTURE.md` for the binding decisions.
