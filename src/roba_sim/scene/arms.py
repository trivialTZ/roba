"""Load and place the two robot arms + the knife end-effector (Aim 1.1, ADR-002).

Cutting arm = OpenArm (Enactic, has a USD). Holding arm = reBot B601 (URDF→USD self-converted).
Each arm becomes an Articulation we can command. The knife is a rigid blade fixed to the cutting
arm's end-effector (neither arm ships a blade).

URDF import note: if an ArmConfig has no ``usd_path``, we import its URDF to USD via the Isaac Sim
URDF importer. That command's exact signature varies by version — adjust ``import_urdf`` for your
build and record it in experiments/ENV.md.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .. import _isaac_compat as ic
from ..config import ArmConfig, BladeConfig

try:
    from pxr import Gf, UsdGeom, UsdPhysics
except Exception:  # pragma: no cover
    Gf = UsdGeom = UsdPhysics = None  # type: ignore


@dataclass
class ArmHandle:
    name: str
    prim_path: str
    ee_prim_path: str
    articulation: object          # Isaac Articulation
    config: ArmConfig


def import_urdf(urdf_path: str, dest_prim: str) -> str:
    """Import a URDF into the current stage, return the resulting prim path.

    Uses the Isaac Sim URDF importer via omni.kit.commands. Fixed base (manipulator), merged
    fixed joints, and position drive are sensible defaults for an arm; tune on SCC.
    """
    import omni.kit.commands  # available inside Isaac Sim

    # Isaac Sim 6.0 importer (isaacsim.asset.importer.urdf 3.x): create the config via _urdf.ImportConfig()
    # — the old "URDFCreateImportConfig" command was removed. Falls back to the legacy command on ≤4.x.
    cfg = None
    try:
        from isaacsim.asset.importer.urdf import _urdf
        cfg = _urdf.ImportConfig()
    except Exception:
        try:
            _status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        except Exception as e:
            raise RuntimeError(f"could not create URDF import config: {e}")

    cfg.merge_fixed_joints = True
    cfg.fix_base = True
    cfg.make_default_prim = False
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    try:
        cfg.default_drive_type = 1      # position drive (field name stable across versions)
    except Exception:
        pass
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=cfg,
        dest_path=dest_prim,
    )
    return dest_prim


def load_arm(arm: ArmConfig, root: str = "/World") -> ArmHandle:
    ic.require_isaac()
    prim_path = f"{root}/{arm.name}"
    if arm.usd_path:
        ic.add_reference_to_stage(usd_path=arm.usd_path, prim_path=prim_path)
    else:
        import_urdf(arm.urdf_path, prim_path)

    articulation = ic.Articulation(prim_paths_expr=prim_path, name=arm.name) \
        if _accepts_expr() else ic.Articulation(prim_path=prim_path, name=arm.name)

    # Place the base.
    _set_world_pose(prim_path, arm.base_position, arm.base_orientation_wxyz)
    ee_prim = f"{prim_path}/{arm.ee_frame}"
    return ArmHandle(arm.name, prim_path, ee_prim, articulation, arm)


def attach_knife(stage, arm: ArmHandle, blade: BladeConfig, name: str = "knife") -> str:
    """Create the blade as a rigid body and fix it to the cutting arm's EE."""
    blade_path = f"{arm.prim_path}/{name}"
    cube = UsdGeom.Cube.Define(stage, blade_path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddScaleOp().Set(Gf.Vec3f(blade.length_m, blade.thickness_m, blade.height_m))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.75, 0.78, 0.82)])
    prim = cube.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.GetMassAttr().Set(blade.mass_kg)

    # Fixed joint EE → blade, offset so the cutting edge sits below the flange.
    joint_path = f"{arm.ee_prim_path}/knife_mount"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.GetBody0Rel().SetTargets([arm.ee_prim_path])
    joint.GetBody1Rel().SetTargets([blade_path])
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(*[-o for o in blade.mount_offset_m]))
    return blade_path


# ---- internal helpers -----------------------------------------------------------------
def _accepts_expr() -> bool:
    """SingleArticulation (5.x) takes prim_paths_expr; legacy Articulation takes prim_path."""
    try:
        import inspect
        return "prim_paths_expr" in inspect.signature(ic.Articulation.__init__).parameters
    except Exception:
        return False


def _set_world_pose(prim_path, position, orientation_wxyz) -> None:
    from pxr import Gf, UsdGeom
    stage = ic.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    w, x, y, z = orientation_wxyz
    xf.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
