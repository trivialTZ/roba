"""Application orchestration — wires Aim 1 together (1.1–1.5).

IMPORTANT: a ``SimulationApp`` must already be instantiated before this module is imported
(Isaac Sim requirement). Use ``experiments/run_demo.py`` as the entry point — it creates the app
first, then constructs ``RobaApp``.

Main loop each step:
    read mode → produce a blade EE target (mouse teleop or an autonomous trajectory)
             → plane-constrained IK → apply to the cutting arm
             → break the seams the blade passed (update the cut)
             → refresh the UI status.
The holding arm stabilizes the workpiece (impedance + weld) the whole time.
"""
from __future__ import annotations

from typing import Iterator, Optional

import numpy as np

from .config import RobaConfig
from .control.cutting_controller import CuttingController
from .control.impedance_hold import ImpedanceHoldController
from .scene.world import RobaWorld
from .teleop.mouse_plane import MousePlaneTeleop
from .teleop.plane_ik import PlaneConstrainedIK

MODE_MANUAL, MODE_SKIVE, MODE_SLICE = 0, 1, 2


class RobaApp:
    def __init__(self, cfg: RobaConfig, sim_app):
        self.cfg = cfg
        self.sim_app = sim_app
        self.world: Optional[RobaWorld] = None
        self.cut: Optional[CuttingController] = None
        self.hold: Optional[ImpedanceHoldController] = None
        self.teleop: Optional[MousePlaneTeleop] = None
        self.ik: Optional[PlaneConstrainedIK] = None
        self.panel = None
        self.playing = False
        self.mode = MODE_MANUAL
        self._t = 0.0
        self._auto: Optional[Iterator] = None

    # ---- setup ------------------------------------------------------------------------
    def setup(self) -> None:
        self.world = RobaWorld(self.cfg)
        self.world.build()

        self.cut = CuttingController(self.cfg.control, self.cfg.pork, self.cfg.blade)
        self.hold = ImpedanceHoldController(self.world.holding_arm, self.cfg.control)
        self.teleop = MousePlaneTeleop(self.cfg.teleop)
        self.ik = PlaneConstrainedIK(self.world.cutting_arm, self.cfg.cutting_arm)
        self.ik.set_base_pose(self.cfg.cutting_arm.base_position,
                              self.cfg.cutting_arm.base_orientation_wxyz)

        # Holding arm: compliant, then weld the nearest slab block (ADR-004).
        self.hold.set_compliance()
        self._weld_holding_arm()

        self._build_ui()

    def _weld_holding_arm(self) -> None:
        # Weld the end column nearest the holding arm so it stabilizes the slab.
        if self.world.slab is None or not self.world.slab._blocks:
            return
        last_ix = max(ix for (ix, _li) in self.world.slab._blocks.keys())
        block = self.world.slab._blocks[(last_ix, 0)]
        self.hold.weld(self.world.stage, block)

    def _build_ui(self) -> None:
        try:
            from .ui.control_panel import RobaControlPanel
            self.panel = RobaControlPanel(self.cfg, callbacks={
                "start": self.start,
                "pause": self.pause,
                "reset": self.reset,
                "set_mode": self.set_mode,
                "set_stiffness": lambda v: self.hold.set_compliance(stiffness=v),
            })
        except Exception as exc:  # headless / no GUI → run without a panel
            print(f"[app] UI panel unavailable ({exc}); running without it")

    # ---- UI callbacks (Aim 1.5) -------------------------------------------------------
    def start(self) -> None:
        self.playing = True
        self._auto = self._make_auto_iter()
        self._status("running")

    def pause(self) -> None:
        self.playing = False
        self._status("paused")

    def reset(self) -> None:
        self.playing = False
        self._t = 0.0
        self._auto = None
        self.cut.reset()
        self.teleop.reset()
        self.world.reset()
        self._status("reset")

    def set_mode(self, mode: int) -> None:
        self.mode = int(mode)
        self._auto = self._make_auto_iter()
        self._status(f"mode → {self.mode}")

    def _make_auto_iter(self):
        if self.mode == MODE_SKIVE:
            return self.cut.skive_trajectory()
        if self.mode == MODE_SLICE:
            return self.cut.slice_trajectory()
        return None

    # ---- main loop --------------------------------------------------------------------
    def run(self) -> None:
        while self.sim_app.is_running():
            self.world.step(render=not self.cfg.sim.headless)
            if self.playing:
                self._control_step()
            else:
                # keep the app responsive (UI, camera) while paused
                pass
        self.sim_app.close()

    def _control_step(self) -> None:
        self._t += self.cfg.sim.physics_dt

        # 1) get a blade EE target from the current source.
        if self.mode == MODE_MANUAL:
            px, pz = self.teleop.read_plane_target()
            pos, quat = self.cut.target_from_plane(px, pz, self._t)
        else:
            try:
                pos, quat = next(self._auto)
            except (StopIteration, TypeError):
                self.pause()
                return

        # 2) plane-constrained IK → apply to the cutting arm.
        action = self.ik.solve_to(pos, quat)
        if action is not None:
            self.world.cutting_arm.articulation.apply_action(action)

        # 3) break the seams the blade has passed (the actual "cut").
        blade_z_bottom = pos[2] + self.cfg.blade.mount_offset_m[2] - self.cfg.blade.height_m / 2
        if self.mode == MODE_SKIVE:
            self.world.slab.break_interface_under(pos[0], blade_z_bottom)
        else:
            self.world.slab.update_cut(pos[0], blade_z_bottom)

        # 4) UI feedback.
        self._status(f"cut {self.world.slab.fraction_cut*100:4.1f}%  ik_fail={self.ik.fail_count}")

    def _status(self, text: str) -> None:
        if self.panel is not None:
            self.panel.set_status(text)
