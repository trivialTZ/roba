# Control Theory & Methods for Robotic Cutting (Aim 2.1 + 2.2)

A literature synthesis of the control theories behind automatic food cutting and surgical
cutting/resection, and the practical roles of force control, impedance/damping control, and visual
servoing. Grounded in the verified research; full sources in `docs/REFERENCES.md` (Pillars 8–9).

---

## 1. Leading control theories & methods (Aim 2.1)

Robotic cutting splits into two communities sharing a control toolkit.

### 1.1 Food / deformable-object cutting
- **Physics-based mechanics modeling.** Fracture-mechanics + FEM force prediction, and the classic
  **slice-push (pressing-slicing) ratio** quantifying how lateral blade motion lowers the normal
  force needed to fracture material. *Leading group:* **Yan-Bin Jia (Iowa State)** — knife-motion
  mechanics, cutting of solids via fracture mechanics + FEM, and T-RO 2024 cutting of fruits/veg.
- **Differentiable simulation.** **DiSECt** (NVIDIA/USC) — a differentiable cutting simulator for
  parameter inference and control; the leading effort to *calibrate* a cutting model to data and
  optimize cutting motions through gradients. **This is our physics-faithful cutting engine (ADR-001).**
- **Learning-based control.** Learning-from-demonstration with **Dynamic Movement Primitives + force
  profiles** (incl. non-holonomic DMPs), differentiable-physics skill learning (**DiffSkill**), and
  full RL + tactile/visual systems (**Sashimi-Bot**, 2025). Data-driven **MPC** handles trajectory
  optimization under deformable dynamics (**Mitsioni/Kragic**).

### 1.2 Surgical cutting / resection
- **STAR program (Axel Krieger, JHU)** is the flagship trajectory: supervised-autonomy suturing
  (2016) → autonomous laparoscopic anastomosis (2022) → **SRT-H (2025)**, a hierarchical
  **language-conditioned imitation-learning** framework that autonomously performed the clip-and-cut
  phase of cholecystectomy on **n=8 ex-vivo porcine** gallbladders at 100% with no human takeover.
  - **Transferable architecture lesson:** a *hierarchical high-level language/task planner over a
    low-level imitation policy, with explicit autonomous error recovery*, is what made long-horizon
    reliability work. Design the meat-cutting agent the same way, not as one monolithic policy.
  - **Caveat:** SRT-H trained on ~18,000 *real* teleop demos on real tissue and runs on real dVRK
    hardware — **not** a sim/sim-to-real result. The sim-to-real gap for deformable cutting (our hard
    part) is exactly what SRT-H sidestepped.
- **Learning-based MPC for tissue manipulation** (Rosen/UCLA Raven), and **autonomous RGB-D-guided
  electrosurgical resection** (JHU) round out the surgical control literature.

### 1.3 Feasibility reality check
Structured food slicing and *supervised* surgical subtasks are demonstrated. Fully autonomous,
unsupervised surgical resection on live tissue remains research-stage with major safety/regulatory
risk. Our project targets the **food** regime in sim, borrowing the surgical *architecture* ideas.

---

## 2. Roles of force / impedance / visual servoing (Aim 2.2)

The three families are **complementary**, not competing; mature systems **layer** them.

### 2.1 Force control (hybrid position/force) — *the contact workhorse*
- Regulates the contact that does the cutting: downward **press force** to fracture material, and
  knife-board contact force. *Canonical example:* **Mu/Xue/Jia (ICRA 2019)** — a 4-DOF Barrett WAM +
  ATI Delta 6-axis F/T sensor cutting onions/potatoes via a press-push-slice hybrid PID-over-force-
  and-position scheme.
- In surgery, force/joint-torque feedback detects when the blade has passed through tissue or keeps
  the knife on a tissue boundary (Edinburgh grapefruit study; PR2 joint-torque classification).
- **Theory:** Raibert–Craig **hybrid position/force** with a selection vector — a *position-vs-force*
  partition of the task DOFs.

### 2.2 Damping / impedance / admittance control — *compliance*
- Imposes spring-mass-damper behavior so the tool **yields** to contact instead of rigidly tracking a
  path. Central to surgeon-cooperative **bone milling** (cooperative impedance control for TKA) and
  learning-based food cutting (**Mitsioni/Kragic** data-driven MPC models velocity-force as mechanical
  impedance; **SliceIt!** fuses impedance + admittance + force control via FDCC on dual UR5e arms).
- **Isaac Sim mapping:** PhysX joints are PD spring-dampers; Isaac Lab's **OSC** does Cartesian
  impedance/variable impedance (closed-loop force on the 3 linear axes only — moment is open-loop;
  Pillar 5).

### 2.3 Visual servoing — *cut geometry / path*
- Guides **where / along what path** to cut on deforming tissue. **STAR** uses image-based visual
  servoing (IBVS) with NIR fluorescent (ICG) markers + dual NIR/RGB cameras to track a cut line;
  **Long et al. (foam cutting)** uses IBVS via image moments to control all 6 DOF of cutting
  depth/angle while a force controller prevents global deformation.

### 2.4 How they combine (the verified nuance)
**Do not hard-code "vision = tangential, force = normal."** That neat split is an *idealization*,
not a law — the most-cited soft-material work (**Long et al.**) deliberately does **not** regulate a
normal-force setpoint (vision controls all 6 DOF incl. depth; force *bounds* global deformation and
adds slicing). Real systems vary: STAR is vision-only during the cut; SliceIt fuses everything via
FDCC + RL. **Design implication for roba:** use a **unified Cartesian impedance/admittance or hybrid
force-velocity controller (FDCC-style)** whose force role is to *bound interaction forces and add
slicing motion*, not track a fixed normal setpoint. RL is a proven way to tune those compliance/
force-modulation parameters in sim before transfer. The most mature deployed combination is
**vision-for-path + force/impedance-for-contact**; pure visual servoing alone is insufficient for
occluded, deforming, contact-rich cuts (the blade buries itself — Pillar 10).

---

## 3. What roba implements from this survey
| Concept | Where in the code |
|---------|-------------------|
| Slice-push ratio (force reduction) | `cutting/material.py`, `control/cutting_controller.py` |
| Hybrid press-and-slice motion | `control/cutting_controller.py` (`target_from_plane`, `slice_trajectory`) |
| Impedance hold (PD spring-damper) | `control/impedance_hold.py` |
| Force bounding (3-axis) | `control/cutting_controller.py` (`_bound_press`) |
| Vision → path prior (not force) | `perception/video2traj/pipeline.py` |
| DiSECt force-model calibration | planned, Phase 2 (ADR-001/005) |
| Hierarchical planner over low-level policy | future RL agent (SRT-H-inspired) |
