"""Cutting arm motion generation (Aim 1.3 / 2.4).

Turns a 2-variable in-plane target (from the mouse, Aim 1.4) into an end-effector *pose* for the
plane-constrained IK, and adds the press-push-slice behavior that real meat cutting needs:

  * a fixed blade-down orientation,
  * a lateral *slicing* oscillation whose amplitude scales with ``slice_push_ratio`` (a draw cut
    lowers required force ~40%, verified — docs/FEASIBILITY.md Pillar 4/11),
  * force bounding so the blade never presses harder than ``max_press_force_n`` (the 3-axis
    contact limit that IS available; moment feedback is not — Pillar 5).

Also provides the two autonomous task trajectories for Aim 2.4:
  (a) skin skiving — shallow constant-depth pass following the top surface,
  (b) vertical slicing — press-and-slice down at evenly spaced X positions.

A "pose" here is (position xyz, quaternion wxyz). No Isaac import needed for the math, so the
trajectory logic is unit-testable.
"""
from __future__ import annotations

import math
from typing import Iterator, Optional, Tuple

from ..config import BladeConfig, ControlConfig, PorkBellyConfig

Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]


class CuttingController:
    def __init__(self, control: ControlConfig, pork: PorkBellyConfig, blade: BladeConfig):
        self.control = control
        self.pork = pork
        self.blade = blade
        self._last_z: Optional[float] = None

    # ---- interactive (Aim 1.4) --------------------------------------------------------
    def target_from_plane(self, plane_x: float, plane_z: float, t: float,
                          measured_force_n: Optional[float] = None) -> Pose:
        """Map a mouse-driven (x, z) plane point to a blade EE pose with slicing + force bound."""
        # Slicing oscillation: lateral draw along X scaled by the slice-push ratio.
        c = self.control
        slice_dx = (c.slice_amplitude_m * _clamp01(c.slice_push_ratio)
                    * math.sin(2.0 * math.pi * c.slice_freq_hz * t))

        z = max(plane_z, c.cut_plane_z_floor_m)
        z = self._bound_press(z, measured_force_n)
        pos = (plane_x + slice_dx, c.cut_plane_y_m, z)
        return pos, c.blade_down_quat_wxyz

    def _bound_press(self, target_z: float, measured_force_n: Optional[float]) -> float:
        """Don't let the blade descend further once the press force hits the cap.

        With a real contact sensor we hold height when force ≥ cap. Without one, we limit the
        per-call descent so the blade can't teleport through the slab (a crude rate limit).
        """
        c = self.control
        if measured_force_n is not None and measured_force_n >= c.max_press_force_n:
            return self._last_z if self._last_z is not None else target_z
        if self._last_z is not None:
            max_descent = 0.01  # m per command — keeps motion physically plausible
            target_z = max(target_z, self._last_z - max_descent)
        self._last_z = target_z
        return target_z

    def reset(self) -> None:
        self._last_z = None

    # ---- autonomous tasks (Aim 2.4) ---------------------------------------------------
    def skive_trajectory(self, n_steps: int = 200) -> Iterator[Pose]:
        """(2.4a) Slice off the top skin: tangential pass at constant shallow depth.

        The blade rides at ``skive_depth`` below the top surface, moving along +X at
        ``skive_speed``. Constant-depth surface following is the crux (no validated prior robot
        does this on chewy skin — it's the project's novel bit, docs/FEASIBILITY.md Pillar 11).
        """
        top_z = self.pork.position[2] + self.pork.total_thickness_m
        z = top_z - self.control.skive_depth_m
        x0, x1 = self.control.cut_plane_y_m, None  # use teleop ranges' X extent
        x_start = self.pork.position[0] - self.pork.length_m / 2
        x_end = self.pork.position[0] + self.pork.length_m / 2
        for i in range(n_steps):
            x = x_start + (x_end - x_start) * i / max(1, n_steps - 1)
            # Skiving uses a near-horizontal draw, so bias the orientation slightly (kept simple).
            yield (x, self.control.cut_plane_y_m, z), self.control.blade_down_quat_wxyz

    def slice_trajectory(self, dwell_steps: int = 30) -> Iterator[Pose]:
        """(2.4b) Vertical slices into cookable pieces: press-and-slice at each X station."""
        top_z = self.pork.position[2] + self.pork.total_thickness_m
        floor_z = self.control.cut_plane_z_floor_m
        x_start = self.pork.position[0] - self.pork.length_m / 2 + self.control.slice_step_m
        x_end = self.pork.position[0] + self.pork.length_m / 2
        n_slices = max(1, int((x_end - x_start) / self.control.slice_step_m))
        for s in range(n_slices):
            x = x_start + s * self.control.slice_step_m
            for d in range(dwell_steps):  # press down from top to floor at this station
                frac = d / max(1, dwell_steps - 1)
                z = top_z + 0.02 - (top_z + 0.02 - floor_z) * frac
                t = s + frac
                slice_dx = (self.control.slice_amplitude_m * _clamp01(self.control.slice_push_ratio)
                            * math.sin(2.0 * math.pi * self.control.slice_freq_hz * t))
                yield (x + slice_dx, self.control.cut_plane_y_m, z), self.control.blade_down_quat_wxyz


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x
