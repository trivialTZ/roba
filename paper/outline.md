# Paper outline (Aim 3.3)

Target: a technical report / cs.RO-style paper (arXiv primary **cs.RO**, cross-list cs.RO/cs.LG;
venues CoRL/ICRA/IROS, ~8 pp double-blind). Working title:

> **roba: A Dual-Arm Robotic Meat-Cutting Simulation in Isaac Sim with Mouse-Teleoperated,
> Plane-Constrained Cutting and a Video-Grounded Control Study**

## Abstract
Dual-arm pork-belly cutting in Isaac Sim; the native-deformable-cutting gap and our breakable-seam +
DiSECt resolution; mouse→plane-IK teleop; a video→kinematics + sensor/sim→force modeling pipeline;
two task controllers (skin skiving, vertical slicing). Honest limitations.

## 1. Introduction
- Motivation: meat cutting now, surgical extension later.
- Contributions: (i) an open dual-arm cutting sim with a layered breakable-seam tissue model;
  (ii) a 2-variable mouse→plane-IK teleop reduction; (iii) a decoupled video→kinematics /
  sensor→force modeling method; (iv) a first **closed-loop constant-depth skin-skiving** controller
  (novel — Pillar 11); (v) an honest account of what Isaac Sim can and cannot do for cutting.

## 2. Related Work
Food-cutting mechanics (Jia, DiSECt); surgical autonomy (STAR/SRT-H); force/impedance/visual
servoing; deformable cutting simulation (FEM/MPM/SOFA). → from `docs/CONTROL_SURVEY.md` + `REFERENCES.md`.

## 3. Background & The Cutting-Simulation Problem
- Isaac Sim/PhysX cannot topologically cut deformables (the verified central constraint).
- Design space: breakable seams vs. DiSECt vs. MPM (Table from `ARCHITECTURE.md` ADR-001).

## 4. System
- 4.1 Platform & arms (OpenArm cutting + reBot holding; ADR-002).
- 4.2 Three-layer breakable-seam pork belly + the toughness→break-force model (`MATERIAL_MODEL.md`).
- 4.3 Dual-arm control: impedance hold + weld; press-push-slice cutting; force bounding.
- 4.4 Mouse → plane-constrained IK teleop (the 2-variable reduction).
- 4.5 UI (omni.ui).

## 5. Data-Driven Control & Modeling
- 5.1 Video → kinematic trajectory prior (decoupled from force; why forces aren't in RGB).
- 5.2 Sensor + DiSECt → calibrated force/cutting model.
- 5.3 Two task controllers: skin skiving (2.4a), vertical slicing (2.4b).

## 6. Experiments
- E1: break-force ordering & slice-push reduction match fracture-mechanics priors (unit-tested).
- E2: teleop tracking quality / IK failure rate on the constrained plane.
- E3: cut-completeness & layer-selectivity for skive vs. slice tasks.
- E4: (if sensor) DiSECt calibration error vs. real force-vs-depth.
- E5: video trajectory-prior fidelity on tutorial clips (qualitative + slice-push estimate).

## 7. Limitations (lead with these — they're the honest contribution)
- Breakable seams cut along predetermined lines, not arbitrary continuum paths.
- Forces from video are not recoverable; force model is sim-only until a sensor is acquired.
- Muscle anisotropy / parameter scatter; literature priors need local calibration.
- Sim-only (no hardware sim-to-real); SCC GPU/driver constraints.
- Skin-skiving validated in sim only; no real-tissue closed-loop baseline exists.

## 8. Conclusion & Future Work
RL policy (SRT-H-style hierarchy); MPM for true severing; real F/T sensor; sim-to-real.

## Reproducibility appendix
Pinned stack (`experiments/ENV.md`), asset-download (no NVIDIA re-hosting), seeds, SCC run kit.
