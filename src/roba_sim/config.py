"""Central configuration for the roba meat-cutting simulation.

All tunable parameters live here as dataclasses so the UI (Aim 1.5) and headless runs can
read/modify a single source of truth. Default material numbers come from
``docs/MATERIAL_MODEL.md`` (order-of-magnitude priors — calibrate locally; see ADR-001).

Nothing here imports Isaac Sim, so this module is safe to import anywhere (tests, CI, the UI
process) without a GPU.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Tuple

# Root under which the open robot assets are mounted inside the Isaac Sim container.
# On SCC the assets live at /projectnb/pi-brout/$USER/roba_work/assets/robots and are bind-mounted
# to /workspace/assets/robots by deploy/scc/run_interactive.sh & job_headless.qsub. Override with
# ROBA_ASSET_ROOT for a different layout.
ASSET_ROOT = os.environ.get("ROBA_ASSET_ROOT", "/workspace/assets/robots")
_OPENARM = f"{ASSET_ROOT}/openarm_isaac_lab/source/openarm/openarm/tasks/manager_based/openarm_manipulation/usds"
_REBOT = f"{ASSET_ROOT}/reBotArmController_ROS2/src/rebotarm_bringup/description/urdf"


# --------------------------------------------------------------------------------------
# Material model (Aim 1.2)
# --------------------------------------------------------------------------------------
@dataclass
class LayerParams:
    """One layer of the pork belly. Fracture toughness drives the cut; modulus drives feel."""

    name: str
    thickness_m: float                 # vertical extent of this layer
    youngs_modulus_pa: float           # elastic stiffness (deformation feel)
    fracture_toughness_j_m2: float     # Jc — energy per unit cut area; sets cutting force
    friction_mu: float                 # blade-tissue friction coefficient (the weak data point)
    density_kg_m3: float
    color: Tuple[float, float, float]  # RGB for visualization


# Defaults: skin ≫ fat > lean in toughness (docs/MATERIAL_MODEL.md).
SKIN = LayerParams("skin", 0.004, 1.0e6, 22_000.0, 0.30, 1100.0, (0.93, 0.85, 0.78))
FAT = LayerParams("fat", 0.012, 5.0e3, 4_100.0, 0.20, 920.0, (0.98, 0.97, 0.90))
LEAN = LayerParams("lean", 0.020, 1.0e5, 600.0, 0.35, 1060.0, (0.78, 0.32, 0.34))


@dataclass
class PorkBellyConfig:
    """A layered slab built from breakable sub-blocks (ADR-001 option b)."""

    layers: List[LayerParams] = field(default_factory=lambda: [LEAN, FAT, SKIN])  # bottom→top
    length_m: float = 0.20             # along the cut direction (X)
    width_m: float = 0.08              # across the cut (Y)
    seam_spacing_m: float = 0.01       # sub-block size → cut resolution along X
    base_height_m: float = 0.02        # height of the cutting board the slab sits on
    position: Tuple[float, float, float] = (0.0, 0.0, 0.02)  # slab origin (on the board)

    @property
    def total_thickness_m(self) -> float:
        return sum(l.thickness_m for l in self.layers)


# --------------------------------------------------------------------------------------
# Robot arms (Aim 1.1) — two distinct arms, ADR-002
# --------------------------------------------------------------------------------------
@dataclass
class ArmConfig:
    name: str
    usd_path: str                      # USD asset path (Nucleus/local); empty → import from URDF
    urdf_path: str                     # fallback URDF to import if usd_path is empty
    ee_frame: str                      # end-effector link/frame name for IK
    base_position: Tuple[float, float, float]
    base_orientation_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    is_cutting: bool = False
    # Lula descriptor + URDF needed by LulaKinematicsSolver (fill once assets are on SCC):
    lula_robot_description: str = ""   # <robot>.yaml (Lula descriptor)
    lula_urdf: str = ""                # URDF used by Lula


# OpenArm = cutting arm (7-DOF, force feedback). reBot = holding arm. Paths are placeholders to
# fill in experiments/ENV.md once assets are pinned on SCC.
# Cutting arm: OpenArm unimanual USD (verified present in enactic/openarm_isaac_lab).
# NOTE: ee_frame is a best guess from the URDF link naming — confirm the exact prim/link name from
# the loaded articulation in Isaac Sim (print articulation.dof_names / body names) and update.
# OpenArm is 7-DOF (joints openarm_joint1..7) + gripper, confirmed by loading the USD on SCC
# (smoke test 2026-06-10). The tool-center-point frame openarm_ee_tcp is the knife mount point.
OPENARM_CUTTING = ArmConfig(
    name="openarm_cutting",
    usd_path=f"{_OPENARM}/openarm_unimanual/openarm_unimanual.usd",
    urdf_path="",  # openarmv2.urdf ships empty; generate from xacro if a URDF is needed for Lula
    ee_frame="openarm_ee_tcp",         # TCP frame (child of openarm_ee_tcp_joint)
    base_position=(0.0, -0.45, 0.0),
    is_cutting=True,
    # Lula needs a descriptor+URDF: generate openarm.urdf from the v2.0 xacro and author the
    # descriptor with Isaac Sim's Lula Robot Description Editor (Phase 1 task).
    lula_robot_description="",
    lula_urdf="",
)
# Holding arm: reBot B601 (with gripper) URDF — self-converted to USD by Isaac's URDF importer.
REBOT_HOLDING = ArmConfig(
    name="rebot_holding",
    usd_path="",  # no official USD (ADR-002) → import the URDF below
    urdf_path=f"{_REBOT}/reBot_B601_DM_with_gripper.urdf",
    ee_frame="gripper_link",           # verified link name in the URDF
    base_position=(0.0, 0.45, 0.0),
    base_orientation_wxyz=(0.0, 0.0, 0.0, 1.0),  # face the workpiece (180° about Z)
    is_cutting=False,
)


# --------------------------------------------------------------------------------------
# Knife end-effector (Aim 1.3) — neither arm ships a blade (ADR-002)
# --------------------------------------------------------------------------------------
@dataclass
class BladeConfig:
    length_m: float = 0.18             # blade length (along X, the slicing axis)
    height_m: float = 0.04             # blade height (Z)
    thickness_m: float = 0.002         # blade thickness (Y)
    edge_radius_m: float = 0.0001      # edge sharpness — smaller = sharper = lower cut force
    mass_kg: float = 0.15
    mount_offset_m: Tuple[float, float, float] = (0.0, 0.0, -0.10)  # blade tip below the EE


# --------------------------------------------------------------------------------------
# Control (Aim 1.3 / 2.4)
# --------------------------------------------------------------------------------------
@dataclass
class ControlConfig:
    # Holding arm impedance (PhysX PD spring-damper, ADR-004)
    hold_stiffness: float = 4000.0     # joint drive stiffness (compliant hold)
    hold_damping: float = 400.0
    weld_after_grasp: bool = True      # rigidly fix workpiece to gripper once grasped

    # Cutting plane (Aim 1.4): blade constrained to a vertical X-Z plane at this Y
    cut_plane_y_m: float = 0.0
    cut_plane_z_floor_m: float = 0.02  # board top — don't cut below this
    blade_down_quat_wxyz: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.0)  # edge down

    # Slice-push (press-vs-slice) ratio: 0 = pure press, 1 = max draw. Lowers cut force ~40%.
    slice_push_ratio: float = 0.6
    slice_amplitude_m: float = 0.03    # lateral draw amplitude for the slicing oscillation
    slice_freq_hz: float = 2.0

    # Force bounding (3-axis contact sensor on the blade, ADR-004)
    max_press_force_n: float = 120.0   # cap downward force to avoid gouging / board damage

    # Autonomous task params (Aim 2.4)
    skive_depth_m: float = 0.003       # constant shallow depth for skin skiving (2.4a)
    skive_speed_m_s: float = 0.02
    slice_step_m: float = 0.012        # spacing between vertical slices (2.4b)


# --------------------------------------------------------------------------------------
# Teleop (Aim 1.4)
# --------------------------------------------------------------------------------------
@dataclass
class TeleopConfig:
    # Mouse-normalized (0..1) maps to this world rectangle on the cutting plane.
    x_range_m: Tuple[float, float] = (-0.12, 0.12)   # along the cut (X)
    z_range_m: Tuple[float, float] = (0.02, 0.14)    # height (Z); floor is the board
    smoothing_alpha: float = 0.25      # EMA on the mouse target (lower = smoother, more lag)
    invert_y: bool = True              # screen-down → world-down


# --------------------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------------------
@dataclass
class SimConfig:
    physics_dt: float = 1.0 / 120.0
    rendering_dt: float = 1.0 / 60.0
    gravity: float = -9.81
    headless: bool = False             # True on SCC batch; False for OnDemand/WebRTC GUI
    backend: str = "physx"             # PhysX, not Newton (ADR-003)


@dataclass
class RobaConfig:
    """Top-level config aggregating everything. Pass one instance around the app."""

    pork: PorkBellyConfig = field(default_factory=PorkBellyConfig)
    cutting_arm: ArmConfig = field(default_factory=lambda: OPENARM_CUTTING)
    holding_arm: ArmConfig = field(default_factory=lambda: REBOT_HOLDING)
    blade: BladeConfig = field(default_factory=BladeConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    teleop: TeleopConfig = field(default_factory=TeleopConfig)
    sim: SimConfig = field(default_factory=SimConfig)


def default_config() -> RobaConfig:
    return RobaConfig()
