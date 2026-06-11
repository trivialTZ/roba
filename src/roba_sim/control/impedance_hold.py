"""Holding arm: compliant grasp + weld of the workpiece (Aim 1.3, ADR-004).

Strategy (verified in docs/FEASIBILITY.md Pillar 5): PhysX joints are PD spring-dampers, so we
set a low-ish stiffness + damping for a *compliant* hold (impedance), move the gripper to a grasp
pose at one end of the slab, then — because closed-loop force grasping of a deformable is not
turn-key — rigidly "weld" the grasped sub-block to the gripper with a fixed joint so the holding
arm stabilizes the workpiece while the other arm cuts. The weld sidesteps reBot's 1.5 kg payload
limit (it carries position, not the full cut reaction).
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .. import _isaac_compat as ic
from ..config import ControlConfig
from ..scene.arms import ArmHandle

try:
    from pxr import Gf, UsdPhysics
except Exception:  # pragma: no cover
    Gf = UsdPhysics = None  # type: ignore


class ImpedanceHoldController:
    def __init__(self, arm: ArmHandle, control: ControlConfig):
        self.arm = arm
        self.control = control
        self._welded = False
        self._grasp_target: Optional[np.ndarray] = None

    def set_compliance(self, stiffness: Optional[float] = None, damping: Optional[float] = None) -> None:
        """Set joint drive gains → impedance behavior (Aim 1.3, live-tunable from the UI)."""
        k = self.control.hold_stiffness if stiffness is None else stiffness
        d = self.control.hold_damping if damping is None else damping
        try:
            ctrl = self.arm.articulation.get_articulation_controller()
            n = self.arm.articulation.num_dof
            ctrl.set_gains(kps=np.full(n, k), kds=np.full(n, d))
        except Exception as exc:  # pragma: no cover — handle/version differences on SCC
            print(f"[impedance_hold] could not set gains ({exc}); set joint drives in USD instead")

    def move_to_grasp(self, joint_positions: np.ndarray) -> None:
        """Command the holding arm to a precomputed grasp configuration.

        joint_positions is provided by the caller (e.g. a one-shot IK at a fixed grasp pose near
        the slab end). Kept as joint-space to avoid a second IK dependency for the holding arm.
        """
        self._grasp_target = np.asarray(joint_positions, dtype=float)
        action = ic.ArticulationAction(joint_positions=self._grasp_target)
        self.arm.articulation.apply_action(action)

    def weld(self, stage, target_block_path: str) -> None:
        """Rigidly fix the grasped sub-block to the gripper once in contact (ADR-004)."""
        if self._welded or not self.control.weld_after_grasp:
            return
        joint_path = f"{self.arm.ee_prim_path}/hold_weld"
        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        joint.GetBody0Rel().SetTargets([self.arm.ee_prim_path])
        joint.GetBody1Rel().SetTargets([target_block_path])
        self._welded = True

    def release(self, stage) -> None:
        joint_path = f"{self.arm.ee_prim_path}/hold_weld"
        prim = stage.GetPrimAtPath(joint_path)
        if prim and prim.IsValid():
            UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
        self._welded = False

    @property
    def is_holding(self) -> bool:
        return self._welded
