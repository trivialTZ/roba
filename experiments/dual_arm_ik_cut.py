"""IK-driven dual-arm cut (Aim 1.3) — the right arm descends the knife into the slab via Jacobian IK.

Self-contained position IK: damped least squares on the articulation's own Jacobian (no URDF/Lula/
cuRobo needed). Loads OpenArm bimanual, attaches a knife to the right TCP, places the 3-layer slab
directly under the right EE, then IK-descends the EE straight down through the slab — reading the
knife's FK each step to drive BreakableSlab.update_cut. Heavily instrumented (Jacobian shape, link
names, convergence) so the indexing is learned in one run. Headless. Output -> $ROBA_OUT.
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

def log(s): print(f"[ik_cut] {s}", flush=True)

try:
    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config
    from roba_sim.cutting.breakable_seams import BreakableSlab

    cfg = default_config()
    asset_root = os.environ.get("ROBA_ASSET_ROOT", "/workspace/assets/robots")
    bimanual = (f"{asset_root}/openarm_isaac_lab/source/openarm/openarm/tasks/"
                f"manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd")

    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt,
                     stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = ic.get_current_stage()
    ic.add_reference_to_stage(usd_path=bimanual, prim_path="/World/OpenArmBi")

    tcp = "/World/OpenArmBi/openarm_right_ee_tcp"
    blade_path = tcp + "/knife"
    c = UsdGeom.Cube.Define(stage, blade_path); c.GetSizeAttr().Set(1.0)
    bxf = UsdGeom.Xformable(c); bxf.ClearXformOpOrder()
    bxf.AddScaleOp().Set(Gf.Vec3f(cfg.blade.length_m, cfg.blade.thickness_m, cfg.blade.height_m))
    pr = c.GetPrim(); UsdPhysics.RigidBodyAPI.Apply(pr); UsdPhysics.CollisionAPI.Apply(pr)
    UsdPhysics.MassAPI.Apply(pr).GetMassAttr().Set(cfg.blade.mass_kg)
    jm = UsdPhysics.FixedJoint.Define(stage, blade_path + "_mount")
    jm.GetBody0Rel().SetTargets([tcp]); jm.GetBody1Rel().SetTargets([blade_path])

    # Use SingleArticulation (ic.Articulation) — PROVEN to move joints in dual_arm_cut.py (the batched
    # isaacsim.core.prims.Articulation view did NOT take effect here).
    view = (ic.Articulation(prim_paths_expr="/World/OpenArmBi")
            if "prim_paths_expr" in ic.Articulation.__init__.__code__.co_varnames
            else ic.Articulation(prim_path="/World/OpenArmBi"))
    try: world.scene.add(view)
    except Exception as e: log(f"scene.add: {e}")
    world.reset()
    for _ in range(10): world.step(render=False)
    try: view.initialize()
    except Exception as e: log(f"init: {e}")

    dof = list(view.dof_names)
    right_idx = [dof.index(f"openarm_right_joint{k}") for k in range(1, 8)]
    results["diag"]["dof_count"] = len(dof)
    results["diag"]["right_idx"] = right_idx

    # FK from the USD transform (proven to track physics headless).
    xcache = UsdGeom.XformCache()
    def wpos(path):
        xcache.Clear()
        t = xcache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)).ExtractTranslation()
        return np.array([float(t[0]), float(t[1]), float(t[2])])

    # NUMERIC Jacobian via finite differences using the PROVEN SingleArticulation interface (1-D arrays).
    q = np.array(view.get_joint_positions()).reshape(-1).astype(float)

    def set_q(qv):
        view.set_joint_positions(qv.reshape(-1))
        try:
            view.apply_action(ic.ArticulationAction(joint_positions=qv.reshape(-1)))
        except Exception:
            pass
        world.step(render=False)

    def settle(n=2):
        for _ in range(n):
            world.step(render=False)

    def numeric_jac(eps=0.02):
        global q
        p0 = wpos(tcp)
        J = np.zeros((3, len(right_idx)))
        for c, j in enumerate(right_idx):
            qp = q.copy(); qp[j] += eps
            set_q(qp)
            J[:, c] = (wpos(tcp) - p0) / eps
            set_q(q)  # restore
        return J, p0

    def ik_step(target_pos, iters=4, lam=0.05, max_dq=0.05):
        global q
        for _ in range(iters):
            J, ee = numeric_jac()
            err = target_pos - ee
            if np.linalg.norm(err) < 1e-4:
                break
            dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(3), err)
            dq = np.clip(dq, -max_dq, max_dq)
            q[right_idx] += dq
            set_q(q)
        return wpos(tcp)

    ee_home = wpos(tcp)
    results["diag"]["ee_home"] = [round(float(v), 4) for v in ee_home]
    log(f"EE home {ee_home}")

    # SANITY: the home pose is at the bottom of the reachable workspace (probe showed all joints raise
    # the EE). So verify IK can move the EE UP 8 cm (the reachable direction).
    reached_up = ik_step(ee_home + np.array([0, 0, 0.08]), iters=25)
    results["diag"]["ee_after_8cm_up_cmd"] = [round(float(v), 4) for v in reached_up]
    log(f"after 8cm-UP cmd: EE={reached_up} (dz={reached_up[2]-ee_home[2]:+.4f})")

    # CUT-READY: raise the EE to a clear height; this becomes the top of the descent.
    ee_top = ik_step(ee_home + np.array([0, 0, 0.10]), iters=30)
    results["diag"]["ee_cut_ready_top"] = [round(float(v), 4) for v in ee_top]
    knife_top = wpos(blade_path)
    tip_top = knife_top[2] - cfg.blade.height_m / 2
    log(f"cut-ready: EE={ee_top}, knife tip z={tip_top:.4f}")

    # Place the slab just below the knife tip at the raised pose, so a descent cuts through it.
    cfg.pork.position = (float(knife_top[0]), float(knife_top[1]),
                         float(tip_top - 0.01 - cfg.pork.total_thickness_m))
    slab = BreakableSlab(stage, cfg.pork, cfg.control, cfg.blade, root="/World/PorkBelly")
    slab.build()
    world.step(render=False)
    log(f"slab origin {cfg.pork.position}")

    # DESCEND from the raised pose back down through the slab (reverse of a reachable motion).
    STEPS = 50
    z_start = wpos(tcp)[2]
    z_end = ee_home[2]                       # descend back toward the reachable home level
    xy = wpos(tcp)[:2]
    for i in range(STEPS):
        tz = z_start + (z_end - z_start) * (i / (STEPS - 1))
        ik_step(np.array([xy[0], xy[1], tz]), iters=3)
        k = wpos(blade_path)
        slab.update_cut(float(k[0]), float(k[2] - cfg.blade.height_m / 2), kerf_m=cfg.pork.seam_spacing_m)
        if i % 5 == 0 or i == STEPS - 1:
            results["trajectory"].append({"step": i, "knife": [round(float(v), 4) for v in k],
                                          "fraction_cut": round(slab.fraction_cut, 4)})

    results["diag"]["final_fraction_cut"] = round(slab.fraction_cut, 4)
    results["diag"]["seams_broken"] = sum(1 for s in slab.seams if s.broken)
    results["ok"] = slab.fraction_cut > 0.0
    log(f"DONE — IK-driven fraction_cut={slab.fraction_cut:.3f}")

except Exception:
    results["errors"].append(traceback.format_exc())
    log("FAILED:"); traceback.print_exc()
finally:
    try:
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "dual_arm_ik_cut_results.json"), "w") as f:
            json.dump(results, f, indent=2)
        log(f"wrote {os.path.join(OUT, 'dual_arm_ik_cut_results.json')}")
    except Exception as e:
        log(f"could not write results: {e}")
    print("IK_CUT_OK" if results["ok"] else "IK_CUT_FAILED", flush=True)
    sim.close()
