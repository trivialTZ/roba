"""Assemble the full simulation world: ground, lighting, dual arms, knife, pork belly.

``RobaWorld`` owns the Isaac Sim ``World`` and exposes the handles the controllers/teleop/UI need.
"""
from __future__ import annotations

from typing import Optional

from .. import _isaac_compat as ic
from ..config import RobaConfig
from .arms import ArmHandle, attach_knife, load_arm
from .pork_belly import build_pork_belly


class RobaWorld:
    def __init__(self, cfg: RobaConfig):
        ic.require_isaac()
        self.cfg = cfg
        self.world = ic.World(
            physics_dt=cfg.sim.physics_dt,
            rendering_dt=cfg.sim.rendering_dt,
            stage_units_in_meters=1.0,
        )
        self.stage = ic.get_current_stage()
        self.cutting_arm: Optional[ArmHandle] = None
        self.holding_arm: Optional[ArmHandle] = None
        self.knife_path: Optional[str] = None
        self.slab = None

    def build(self) -> None:
        self.world.scene.add_default_ground_plane()

        # Arms (Aim 1.1): OpenArm cuts, reBot holds (ADR-002).
        self.cutting_arm = load_arm(self.cfg.cutting_arm)
        self.holding_arm = load_arm(self.cfg.holding_arm)
        self.knife_path = attach_knife(self.stage, self.cutting_arm, self.cfg.blade)

        # Workpiece (Aim 1.2): layered breakable slab on a board.
        self.slab = build_pork_belly(
            self.stage, self.cfg.pork, self.cfg.control, self.cfg.blade
        )

        self.world.reset()  # initialize physics handles for the articulations

    def reset(self) -> None:
        """Reset the scene and re-bond all seams (Aim 1.5 'reset')."""
        self.world.reset()
        if self.slab is not None:
            self.slab.reset()

    def step(self, render: bool = True) -> None:
        self.world.step(render=render)
