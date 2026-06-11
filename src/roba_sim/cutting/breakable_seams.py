"""Pre-scored breakable-seam pork belly (ADR-001 option b).

Isaac Sim / PhysX cannot topologically cut a deformable continuum (see docs/FEASIBILITY.md
Pillar 3), so we author the slab as a grid of rigid sub-blocks joined by *breakable* fixed
joints along candidate cut lines. A descending blade separates columns by disabling the seam
joints it passes — deterministic and real-time. Each seam's strength comes from the layer's
fracture toughness via ``material.seam_break_force_n`` so skin resists far more than fat/lean.

Built with the stable ``pxr`` USD/PhysX schema so it survives Isaac Sim version churn. The only
Isaac-specific assumption is that a ``Usd.Stage`` is available (it is, inside a running app).

Coordinate convention (matches config.py): X = along the cut, Y = across the cut (slab width),
Z = up (layers stack in Z). Seams that a vertical cut must break are the X-adjacent joints.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from ..config import BladeConfig, ControlConfig, PorkBellyConfig
from .material import layer_at_height, seam_break_force_n

try:  # pxr is only present inside Isaac Sim / a USD runtime
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
except Exception:  # pragma: no cover - allows importing/test on a plain machine
    Gf = Usd = UsdGeom = UsdPhysics = UsdShade = None  # type: ignore


class SeamJoint:
    """Bookkeeping for one breakable seam between two X-adjacent sub-blocks."""

    __slots__ = ("prim_path", "x_center_m", "z_center_m", "layer_name", "broken", "_joint")

    def __init__(self, prim_path, x_center_m, z_center_m, layer_name, joint):
        self.prim_path = prim_path
        self.x_center_m = x_center_m
        self.z_center_m = z_center_m
        self.layer_name = layer_name
        self.broken = False
        self._joint = joint  # UsdPhysics.Joint

    def break_now(self) -> None:
        if self.broken or self._joint is None:
            return
        # Disabling the joint severs the bond instantly and reversibly (re-enable on reset).
        self._joint.GetJointEnabledAttr().Set(False)
        self.broken = True

    def reset(self) -> None:
        if self._joint is not None:
            self._joint.GetJointEnabledAttr().Set(True)
        self.broken = False


class BreakableSlab:
    """A layered pork-belly slab of breakable sub-blocks.

    Usage (inside a running Isaac Sim app, after a World/Stage exists)::

        slab = BreakableSlab(stage, pork_cfg, control_cfg, blade_cfg, root="/World/PorkBelly")
        slab.build()
        ...
        # each physics step, given the blade tip pose in world coords:
        slab.update_cut(blade_tip_x, blade_tip_z_bottom)
    """

    def __init__(
        self,
        stage: "Usd.Stage",
        pork: PorkBellyConfig,
        control: ControlConfig,
        blade: BladeConfig,
        root: str = "/World/PorkBelly",
    ):
        if Usd is None:
            raise RuntimeError("pxr not available — BreakableSlab must run inside Isaac Sim / a USD runtime")
        self.stage = stage
        self.pork = pork
        self.control = control
        self.blade = blade
        self.root = root
        self.seams: List[SeamJoint] = []
        # block grid indexed by (ix, layer_index) → prim path
        self._blocks: Dict[Tuple[int, int], str] = {}
        self._n_cols = max(1, round(pork.length_m / pork.seam_spacing_m))

    # ---- construction -----------------------------------------------------------------
    def build(self) -> None:
        UsdGeom.Xform.Define(self.stage, self.root)
        ox, oy, oz = self.pork.position
        sx = self.pork.length_m / self._n_cols            # sub-block size along X
        sy = self.pork.width_m                            # full width across Y
        for ix in range(self._n_cols):
            x = ox - self.pork.length_m / 2 + (ix + 0.5) * sx
            z_local = 0.0
            for li, layer in enumerate(self.pork.layers):
                sz = layer.thickness_m
                z = oz + z_local + sz / 2
                path = f"{self.root}/blk_{ix}_{li}"
                self._make_block(path, (x, oy, z), (sx, sy, sz), layer)
                self._blocks[(ix, li)] = path
                z_local += sz

        self._make_seams(sx)
        # Inter-layer seams let the skin peel off the fat for skin-skiving (Aim 2.4a).
        self._make_interlayer_seams()

    def _make_block(self, path, center, size, layer) -> None:
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddScaleOp().Set(Gf.Vec3f(size[0], size[1], size[2]))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*layer.color)])
        prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.GetDensityAttr().Set(layer.density_kg_m3)

    def _make_seams(self, sx: float) -> None:
        """Breakable X-adjacent fixed joints — these are what a vertical cut separates."""
        ox, oy, oz = self.pork.position
        for ix in range(self._n_cols - 1):
            z_local = 0.0
            for li, layer in enumerate(self.pork.layers):
                sz = layer.thickness_m
                z = oz + z_local + sz / 2
                x_seam = ox - self.pork.length_m / 2 + (ix + 1) * sx
                a = self._blocks[(ix, li)]
                b = self._blocks[(ix + 1, li)]
                path = f"{self.root}/seam_x_{ix}_{li}"
                fbreak = seam_break_force_n(layer, self.pork.width_m, self.control, self.blade)
                joint = self._fixed_joint(path, a, b, fbreak)
                self.seams.append(SeamJoint(path, x_seam, z, layer.name, joint))
                z_local += sz

    def _make_interlayer_seams(self) -> None:
        """Breakable Z-adjacent joints so the top (skin) layer can be skived off the fat."""
        oz = self.pork.position[2]
        for ix in range(self._n_cols):
            z_local = 0.0
            for li in range(len(self.pork.layers) - 1):
                lower = self.pork.layers[li]
                z_local += lower.thickness_m
                a = self._blocks[(ix, li)]
                b = self._blocks[(ix, li + 1)]
                path = f"{self.root}/seam_z_{ix}_{li}"
                # Interface toughness ≈ the weaker (adhesion) bond; use the lower layer's Jc.
                fbreak = seam_break_force_n(lower, self.pork.seam_spacing_m, self.control, self.blade)
                self._fixed_joint(path, a, b, fbreak)
                # interlayer seams are tracked separately for the skive task
                self.seams.append(SeamJoint(path, self._blocks_x(ix), oz + z_local,
                                            f"{lower.name}/{self.pork.layers[li+1].name}", self._last_joint))

    def _fixed_joint(self, path, body_a, body_b, break_force_n: float):
        joint = UsdPhysics.FixedJoint.Define(self.stage, path)
        joint.GetBody0Rel().SetTargets([body_a])
        joint.GetBody1Rel().SetTargets([body_b])
        joint.GetJointEnabledAttr().Set(True)
        # PhysX honors physics:breakForce/breakTorque for force-driven realism; we also break
        # geometrically (update_cut) so the demo is deterministic even if contact is finicky.
        joint.CreateBreakForceAttr(float(break_force_n))
        joint.CreateBreakTorqueAttr(float(break_force_n) * 0.5)
        self._last_joint = joint
        return joint

    def _blocks_x(self, ix: int) -> float:
        sx = self.pork.length_m / self._n_cols
        return self.pork.position[0] - self.pork.length_m / 2 + (ix + 0.5) * sx

    # ---- runtime ----------------------------------------------------------------------
    def update_cut(self, blade_x_m: float, blade_z_bottom_m: float, kerf_m: float = 0.006) -> int:
        """Break every X-seam the blade has descended past at column ``blade_x_m``.

        A seam breaks when the blade is horizontally within ``kerf_m`` of it AND the blade tip
        is at or below the seam's center height (i.e. the edge has reached that layer). Returns
        the number of seams broken this call (useful for a force/feedback signal).
        """
        broken = 0
        for s in self.seams:
            if s.broken:
                continue
            if abs(s.x_center_m - blade_x_m) <= kerf_m and blade_z_bottom_m <= s.z_center_m:
                s.break_now()
                broken += 1
        return broken

    def break_interface_under(self, blade_x_m: float, depth_z_m: float,
                              kerf_m: float = 0.006, z_tol_m: float = 0.006) -> int:
        """For skin-skiving (2.4a): break only the inter-layer seam nearest the skive depth.

        A tangential cut at height ``depth_z_m`` severs the adhesion interface within ``z_tol_m`` of
        that height (e.g. the skin/fat bond) along the blade's x path, leaving deeper interfaces and
        the in-layer (column) seams intact — so the skin peels off as a connected sheet.
        """
        broken = 0
        for s in self.seams:
            if s.broken or "/" not in s.layer_name:  # only interlayer seams have '/' in the name
                continue
            if abs(s.x_center_m - blade_x_m) <= kerf_m and abs(s.z_center_m - depth_z_m) <= z_tol_m:
                s.break_now()
                broken += 1
        return broken

    def reset(self) -> None:
        for s in self.seams:
            s.reset()

    @property
    def fraction_cut(self) -> float:
        if not self.seams:
            return 0.0
        return sum(1 for s in self.seams if s.broken) / len(self.seams)
