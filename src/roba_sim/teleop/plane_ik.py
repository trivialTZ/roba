"""Plane-constrained inverse kinematics for the cutting arm (Aim 1.4).

Wraps Lula (``LulaKinematicsSolver`` + ``ArticulationKinematicsSolver``) — the same solver behind
RMPflow — to drive the blade to a target on the cutting plane with a fixed downward orientation.
Per the verified guidance (docs/FEASIBILITY.md Pillar 6), for a constrained 2-DOF target the
warm-started Lula IK is smooth enough *if* you (1) always check the success flag and hold the last
good joint solution on failure (kills the biggest jump source near singularities), and (2) feed a
smoothed target (done upstream in mouse_plane). RMPflow is reserved for the autonomous cutting
task, not this teleop demo.

Namespace note: motion_generation moved from ``omni.isaac.motion_generation`` to
``isaacsim.robot_motion.motion_generation``. Resolved below; adjust on your SCC build if needed.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

try:
    try:
        from isaacsim.robot_motion.motion_generation import (  # type: ignore
            ArticulationKinematicsSolver, LulaKinematicsSolver,
        )
    except Exception:
        from omni.isaac.motion_generation import (  # type: ignore
            ArticulationKinematicsSolver, LulaKinematicsSolver,
        )
    _MG_AVAILABLE = True
except Exception:  # pragma: no cover
    ArticulationKinematicsSolver = LulaKinematicsSolver = None  # type: ignore
    _MG_AVAILABLE = False

from ..config import ArmConfig
from ..scene.arms import ArmHandle


class PlaneConstrainedIK:
    def __init__(self, arm: ArmHandle, arm_cfg: ArmConfig):
        if not _MG_AVAILABLE:
            raise RuntimeError("Lula motion_generation not available — run inside Isaac Sim")
        self.arm = arm
        self._lula = LulaKinematicsSolver(
            robot_description_path=arm_cfg.lula_robot_description,
            urdf_path=arm_cfg.lula_urdf,
        )
        self._ik = ArticulationKinematicsSolver(
            arm.articulation, self._lula, arm_cfg.ee_frame
        )
        self._last_good_action = None
        self.fail_count = 0

    def set_base_pose(self, position, orientation_wxyz) -> None:
        """Tell Lula where the arm base is in the world (IK is solved in base frame)."""
        self._lula.set_robot_base_pose(
            np.asarray(position, dtype=float),
            np.asarray(orientation_wxyz, dtype=float),
        )

    def solve_to(self, position: Tuple[float, float, float],
                 orientation_wxyz: Tuple[float, float, float, float]):
        """Solve IK to a plane target. Returns an ArticulationAction or the last-good on failure.

        Always returns *something* safe to apply: on IK failure we re-issue the last successful
        action, so the arm holds rather than jumping.
        """
        action, success = self._ik.compute_inverse_kinematics(
            target_position=np.asarray(position, dtype=float),
            target_orientation=np.asarray(orientation_wxyz, dtype=float),
        )
        if success:
            self._last_good_action = action
            self.fail_count = 0
            return action
        self.fail_count += 1
        return self._last_good_action  # may be None on the very first frame; caller guards

    def blade_tip_world(self, blade_mount_offset_m) -> Optional[np.ndarray]:
        """Forward-kinematics the current EE pose and offset to the blade tip (for cut updates)."""
        try:
            pos, _ = self._ik.compute_end_effector_pose()
            return np.asarray(pos, dtype=float) + np.asarray(blade_mount_offset_m, dtype=float)
        except Exception:
            return None
