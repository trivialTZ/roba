"""Robot-driven dual-arm cut (Aim 1.3) — the cutting arm plunges the knife through the slab.

No IK yet: we (1) load the OpenArm bimanual USD + attach a knife to the right TCP, (2) probe each
right-arm joint to find the one that lowers the knife most, (3) position the 3-layer slab directly
under the knife's real world pose, (4) ramp that joint to plunge the knife down, reading the knife's
true FK position each step to drive BreakableSlab.update_cut. Records the knife trajectory so the run
is diagnostic even if the geometry needs tuning. Left arm = holder (posed). Headless. Output->$ROBA_OUT.
"""
import json
import os
import sys
import traceback

try:
    from isaacsim import SimulationApp
except Exception:
    from omni.isaac.kit import SimulationApp

sim = SimulationApp({"headless": True})

OUT = os.environ.get("ROBA_OUT", "/tmp")
results = {"ok": False, "diag": {}, "trajectory": [], "errors": []}

def log(s): print(f"[dual_arm_cut] {s}", flush=True)

try:
    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config
    from roba_sim.cutting.breakable_seams import BreakableSlab
    from isaacsim.core.prims import RigidPrim

    cfg = default_config()
    asset_root = os.environ.get("ROBA_ASSET_ROOT", "/workspace/assets/robots")
    bimanual = (f"{asset_root}/openarm_isaac_lab/source/openarm/openarm/tasks/"
                f"manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd")

    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt,
                     stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = ic.get_current_stage()
    ic.add_reference_to_stage(usd_path=bimanual, prim_path="/World/OpenArmBi")

    # knife on the right TCP
    tcp = "/World/OpenArmBi/openarm_right_ee_tcp"
    blade_path = tcp + "/knife"
    c = UsdGeom.Cube.Define(stage, blade_path); c.GetSizeAttr().Set(1.0)
    bxf = UsdGeom.Xformable(c); bxf.ClearXformOpOrder()
    bxf.AddScaleOp().Set(Gf.Vec3f(cfg.blade.length_m, cfg.blade.thickness_m, cfg.blade.height_m))
    prim = c.GetPrim(); UsdPhysics.RigidBodyAPI.Apply(prim); UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(cfg.blade.mass_kg)
    jm = UsdPhysics.FixedJoint.Define(stage, blade_path + "_mount")
    jm.GetBody0Rel().SetTargets([tcp]); jm.GetBody1Rel().SetTargets([blade_path])

    # Create the articulation and REGISTER it with the world scene so reset() initializes its
    # physics view (dof_names works from metadata, but get_joint_positions needs the live view).
    art = (ic.Articulation(prim_paths_expr="/World/OpenArmBi")
           if "prim_paths_expr" in ic.Articulation.__init__.__code__.co_varnames
           else ic.Articulation(prim_path="/World/OpenArmBi"))
    try:
        world.scene.add(art)
    except Exception as e:
        log(f"scene.add(art): {e}")
    world.reset()
    for _ in range(10):
        world.step(render=False)
    try:
        art.initialize()
    except Exception as e:
        log(f"art.initialize(): {e}")

    dof = list(art.dof_names)
    right_idx = [dof.index(f"openarm_right_joint{k}") for k in range(1, 8)]
    q0 = np.array(art.get_joint_positions()).reshape(-1).astype(float)
    results["diag"]["q0_size"] = int(q0.size)
    results["diag"]["dof_count"] = len(dof)
    if q0.size != len(dof):
        raise RuntimeError(f"articulation view not initialized: q0 size {q0.size} != dof {len(dof)}")

    # The knife is fixed-jointed to the TCP, so it is now part of the articulation (a RigidPrim
    # velocity view fails on it). Read its world pose from the USD transform instead. Record the
    # articulation's link API + whether the transform actually tracks physics (probe dz != 0).
    results["diag"]["art_linkish_methods"] = [
        m for m in dir(art) if any(k in m.lower() for k in ("link", "body", "transform"))]
    xcache = UsdGeom.XformCache()
    knife_prim = stage.GetPrimAtPath(blade_path)
    def kpos():
        xcache.Clear()
        t = xcache.GetLocalToWorldTransform(knife_prim).ExtractTranslation()
        return np.array([float(t[0]), float(t[1]), float(t[2])], dtype=float)

    base = kpos().copy()
    results["diag"]["knife_default_pos"] = [round(float(v), 4) for v in base]
    log(f"knife default world pos = {base}")

    # probe: which right joint lowers the knife most? (also tells us if the USD transform tracks physics)
    best_j, best_dz, probe_dz = right_idx[1], 0.0, {}
    for j in right_idx:
        q = q0.copy(); q[j] += 0.3
        art.set_joint_positions(q)
        for _ in range(4):
            world.step(render=False)
        dz = float(kpos()[2] - base[2])
        probe_dz[dof[j]] = round(dz, 4)
        log(f"  probe joint dof#{j} ({dof[j]}): dz={dz:+.4f}")
        if dz < best_dz:
            best_dz, best_j = dz, j
        art.set_joint_positions(q0)
        for _ in range(4):
            world.step(render=False)
    results["diag"]["probe_dz"] = probe_dz
    results["diag"]["plunge_joint"] = dof[best_j]
    results["diag"]["plunge_joint_dz_at_0.3"] = round(best_dz, 4)
    log(f"plunge joint = {dof[best_j]} (dz={best_dz:+.4f} at +0.3 rad)")

    # position the slab under the knife's default pose, top ~2 cm below the knife tip
    art.set_joint_positions(q0)
    for _ in range(6):
        world.step(render=False)
    kp = kpos()
    tip_z = float(kp[2] - cfg.blade.height_m / 2)
    cfg.pork.position = (float(kp[0]), float(kp[1]), tip_z - 0.02 - cfg.pork.total_thickness_m)
    log(f"slab origin set to {cfg.pork.position} (under knife tip z={tip_z:.4f})")

    slab = BreakableSlab(stage, cfg.pork, cfg.control, cfg.blade, root="/World/PorkBelly")
    slab.build()
    world.reset()
    for _ in range(6):
        world.step(render=False)

    # plunge: ramp the chosen joint, drive the cut from the knife's real FK position
    PLUNGE = 0.9          # rad swept on the plunge joint
    STEPS = 120
    for i in range(STEPS):
        q = q0.copy()
        q[best_j] += PLUNGE * (i / (STEPS - 1))
        art.apply_action(ic.ArticulationAction(joint_positions=q))
        world.step(render=False)
        p = kpos()
        z_bottom = float(p[2] - cfg.blade.height_m / 2)
        slab.update_cut(float(p[0]), z_bottom, kerf_m=cfg.pork.seam_spacing_m)
        if i % 8 == 0 or i == STEPS - 1:
            results["trajectory"].append({"step": i, "knife": [round(float(v), 4) for v in p],
                                          "fraction_cut": round(slab.fraction_cut, 4)})

    results["diag"]["final_fraction_cut"] = round(slab.fraction_cut, 4)
    results["diag"]["seams_broken"] = sum(1 for s in slab.seams if s.broken)
    results["ok"] = slab.fraction_cut > 0.0
    log(f"DONE — robot-driven fraction_cut={slab.fraction_cut:.3f}, "
        f"knife z {base[2]:.3f}->{kpos()[2]:.3f}")

except Exception:
    results["errors"].append(traceback.format_exc())
    log("FAILED:"); traceback.print_exc()
finally:
    try:
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "dual_arm_cut_results.json"), "w") as f:
            json.dump(results, f, indent=2)
        log(f"wrote {os.path.join(OUT, 'dual_arm_cut_results.json')}")
    except Exception as e:
        log(f"could not write results: {e}")
    print("DUAL_ARM_CUT_OK" if results["ok"] else "DUAL_ARM_CUT_FAILED", flush=True)
    sim.close()
