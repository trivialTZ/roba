# Environment — pinned versions (reproducibility record)

Fill in the blanks during Phase 0.1 and update whenever the stack changes. Isaac Sim/Lab break
silently across versions, so pin everything. (See `docs/ARCHITECTURE.md` ADR-007.)

## Compute host — BU SCC (verified 2026-06-10)
- Cluster: **BU SCC** · project: **`pi-brout`** · user: `tztang`
- Scratch root: `/projectnb/pi-brout/tztang/roba_work` (home is 10 GB-capped, ~6.4 GB used — keep big files in scratch)
- Repo on SCC: `/projectnb/pi-brout/tztang/roba` (rsync target)
- Container tooling: **`singularity` / `scc-singularity`** (on PATH, no module; NOT apptainer)
- Login-node Python for GPU-free tests: `module load python3/3.10.12` (default `python3` is 3.6 — too old)
- GPU requested: **A40** (alt A6000 / L40S ×29 / RTX6000ada). NEVER A100/H200/V100/P100 (no RT cores)
- **GPU-node driver = `595.71.05`** (L40S nodes; modern RT-core nodes are uniform). Satisfies NVIDIA's
  stated ≥580.65.06 requirement, BUT see the rendering note below.
- Mac role: thin client only (browser/VNC/SSH); cannot host Isaac Sim
- ✅ GPU-free core tests pass on SCC (`python3 tests/test_core.py` → 7/7) and on the Mac

## ⚠️ Rendering status (verified 2026-06-10)
- ✅ **Isaac Sim 6.0 runs HEADLESS on SCC** (physics + USD + articulation) — validated: loaded
  OpenArm USD, built world, read `dof_names`. Requires the headless workaround (now in
  `deploy/scc/job_headless.qsub`): writable `kit/cache`+`kit/logs` binds + `__GLX_VENDOR_LIBRARY_NAME=nvidia`
  + `VK_ICD_FILENAMES`. Without it, SimulationApp **segfaults** in the GLX/EGL stack.
- ❌ **RTX rendering FAILS** on driver 595.71.05: `vkCreateDevice → ERROR_INITIALIZATION_FAILED`
  (GitHub IsaacSim#537: 595 driver breaks Isaac 5.1/6.0; works on 580). So the **interactive GUI demo,
  WebRTC livestream, and camera/RTX sensors are blocked** until a ~580–585-driver GPU node is available.
  **Action: ask BU RCS** whether any GPU node runs a ≤585 driver, or for guidance. Headless physics/RL
  development proceeds without it. (`--nvccli` not configured on SCC; `vulkaninfo` not in the container.)

## Pinned stack
| Component | Version | Notes |
|-----------|---------|-------|
| Isaac Sim | `6.0.0` (or `5.1.0`) | NGC `nvcr.io/nvidia/isaac-sim:<ver>`; built to `roba_work/sif/` |
| Isaac Lab | `__________` | match to Isaac Sim version |
| Python (in container) | `3.11` | container-provided |
| CUDA / driver | `__________` | host driver via `nvidia-smi` |
| Singularity | `__________` | `singularity --version` on SCC |
| DiSECt | commit `__________` | physics-faithful cutting (ADR-001), Phase 2 |

## Assets (cloned to SCC 2026-06-10, in scratch — bind-mounted to /workspace/assets in the container)
- **Cutting arm — OpenArm USD:** `openarm_isaac_lab/.../usds/openarm_unimanual/openarm_unimanual.usd` ✅ loads in Isaac
  - **CONFIRMED 7-DOF**: dof_names = `openarm_joint1..7` + `openarm_finger_joint1/2`, `openarm_hand_joint`, `openarm_ee_tcp_joint`
  - **EE frame = `openarm_ee_tcp`** (TCP, child of openarm_ee_tcp_joint) — set in config.py ✅
  - bimanual also available: `.../openarm_bimanual/openarm_bimanual.usd`
  - `openarmv2.urdf` ships **empty**; generate from `assets/robot/openarm_v2.0/urdf/*.xacro` if a URDF is needed for Lula IK
  - NOTE: `pxr` is NOT importable via bare `/isaac-sim/python.sh` in 6.0 — introspect via a running SimulationApp instead
- **Holding arm — reBot B601 URDF:** `reBotArmController_ROS2/.../urdf/reBot_B601_DM_with_gripper.urdf` ✅ present (563 lines)
  - links: `base_link, link1..link6, gripper_link, gripper_left/right`; joints `joint1..6` (revolute), gripper prismatic
  - EE frame: **`gripper_link`** ✅ verified
- Repo commits to pin: openarm_isaac_lab `__________`, openarm_description `__________`, reBotArmController_ROS2 `__________`

## Image digest
- `.sif` path: `/projectnb/pi-brout/tztang/roba_work/sif/isaac-sim-<ver>.sif`
- source image digest: `__________` (from `singularity inspect`)

## RNG seeds
- Global seed(s): `__________` · note: GPU/PhysX is only *approximately* reproducible.

## What actually worked on SCC (notes from Phase 0.1)
- Container build command tweaks: `__________`
- Connectivity for interactive GUI (OnDemand desktop / WebRTC port-forward): `__________`
- Singularity bind-set adjustments needed: `__________`
- Confirmed OpenArm EE prim name: `__________`
