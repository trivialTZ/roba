"""Pork-belly workpiece: a cutting board + the layered breakable slab (Aim 1.2)."""
from __future__ import annotations

from .. import _isaac_compat as ic
from ..config import BladeConfig, ControlConfig, PorkBellyConfig
from ..cutting.breakable_seams import BreakableSlab

try:
    from pxr import Gf, UsdGeom, UsdPhysics
except Exception:  # pragma: no cover
    Gf = UsdGeom = UsdPhysics = None  # type: ignore


def build_pork_belly(
    stage,
    pork: PorkBellyConfig,
    control: ControlConfig,
    blade: BladeConfig,
    root: str = "/World/PorkBelly",
) -> BreakableSlab:
    """Create the board and the layered slab; return the slab for runtime cut updates."""
    _build_board(stage, pork)
    slab = BreakableSlab(stage, pork, control, blade, root=root)
    slab.build()
    return slab


def _build_board(stage, pork: PorkBellyConfig) -> None:
    """A static cutting board under the slab (the slab rests on it)."""
    path = "/World/CuttingBoard"
    ox, oy, oz = pork.position
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    # Board centered under the slab, top surface at z = oz (slab origin).
    xf.AddTranslateOp().Set(Gf.Vec3d(ox, oy, oz - pork.base_height_m / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(pork.length_m * 1.6, pork.width_m * 1.8, pork.base_height_m))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.45, 0.30, 0.18)])
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)  # static collider (no RigidBodyAPI → immovable)
