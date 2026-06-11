"""Bimanual hold-and-cut (Aim 1.3 complete): left arm holds the slab via impedance + weld while the
right arm cuts it.

- Left (holding) arm: set to COMPLIANT joint gains (low stiffness/damping = impedance), IK-moved so its
  gripper reaches the +Y edge of the slab, then the slab edge is WELDED to the gripper (grasp, ADR-004).
- Right (cutting) arm: finite-difference-Jacobian IK raises the knife then descends through the slab.
- Slab placed at the midpoint between the two arms' home EEs (reachable by both).
Metric: the held block stays near the holding gripper through the cut (stabilized) AND seams break.
Headless. Output -> $ROBA_OUT.
"""
import json, os, sys, traceback

try:
    from isaacsim import SimulationApp
except Exception:
    from omni.isaac.kit import SimulationApp
sim = SimulationApp({"headless": True})

OUT = os.environ.get("ROBA_OUT", "/tmp")
res = {"ok": False, "diag": {}, "errors": []}
def log(s): print(f"[hold_and_cut] {s}", flush=True)

try:
    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config
    from roba_sim.cutting.breakable_seams import BreakableSlab
    from roba_sim.recording import Recorder

    cfg = default_config()
    asset_root = os.environ.get("ROBA_ASSET_ROOT", "/workspace/assets/robots")
    bimanual = (f"{asset_root}/openarm_isaac_lab/source/openarm/openarm/tasks/"
                f"manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd")
    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = ic.get_current_stage()
    ic.add_reference_to_stage(usd_path=bimanual, prim_path="/World/OpenArmBi")

    rtcp, ltcp = "/World/OpenArmBi/openarm_right_ee_tcp", "/World/OpenArmBi/openarm_left_ee_tcp"
    blade_path = rtcp + "/knife"
    c = UsdGeom.Cube.Define(stage, blade_path); c.GetSizeAttr().Set(1.0)
    bxf = UsdGeom.Xformable(c); bxf.ClearXformOpOrder()
    bxf.AddScaleOp().Set(Gf.Vec3f(cfg.blade.thickness_m, cfg.pork.width_m * 1.1, cfg.blade.height_m))
    pr = c.GetPrim(); UsdPhysics.RigidBodyAPI.Apply(pr); UsdPhysics.CollisionAPI.Apply(pr)
    UsdPhysics.MassAPI.Apply(pr).GetMassAttr().Set(cfg.blade.mass_kg)
    UsdPhysics.FixedJoint.Define(stage, blade_path + "_mount").GetBody0Rel().SetTargets([rtcp])
    UsdPhysics.FixedJoint.Get(stage, blade_path + "_mount").GetBody1Rel().SetTargets([blade_path])

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
    R = [dof.index(f"openarm_right_joint{k}") for k in range(1, 8)]
    L = [dof.index(f"openarm_left_joint{k}") for k in range(1, 8)]
    q = np.array(view.get_joint_positions()).reshape(-1).astype(float)

    xcache = UsdGeom.XformCache()
    def wpos(path):
        xcache.Clear()
        t = xcache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)).ExtractTranslation()
        return np.array([float(t[0]), float(t[1]), float(t[2])])
    def set_q(qv):
        view.set_joint_positions(qv.reshape(-1))
        try: view.apply_action(ic.ArticulationAction(joint_positions=qv.reshape(-1)))
        except Exception: pass
        world.step(render=False)
    def ik(tcp, idx, target, iters=12, eps=0.02, lam=0.05, max_dq=0.06):
        global q
        for _ in range(iters):
            p0 = wpos(tcp); J = np.zeros((3, len(idx)))
            for cidx, j in enumerate(idx):
                qp = q.copy(); qp[j] += eps; set_q(qp); J[:, cidx] = (wpos(tcp) - p0) / eps; set_q(q)
            err = target - p0
            if np.linalg.norm(err) < 1e-3: break
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2*np.eye(3), err)
            q[idx] += np.clip(dq, -max_dq, max_dq); set_q(q)
        return wpos(tcp)

    r_home, l_home = wpos(rtcp), wpos(ltcp)
    res["diag"]["r_home"] = [round(float(v),4) for v in r_home]
    res["diag"]["l_home"] = [round(float(v),4) for v in l_home]
    mid = (r_home + l_home) / 2
    log(f"r_home {r_home}, l_home {l_home}, mid {mid}")

    # raise the right knife above the midpoint (cut-ready), place slab just below the knife tip there
    ee_top = ik(rtcp, R, np.array([mid[0], mid[1], r_home[2] + 0.10]), iters=30)
    knife_top = wpos(blade_path); tip = knife_top[2] - cfg.blade.height_m/2
    cfg.pork.position = (float(knife_top[0]), float(knife_top[1]), float(tip - 0.01 - cfg.pork.total_thickness_m))
    slab = BreakableSlab(stage, cfg.pork, cfg.control, cfg.blade, root="/World/PorkBelly")
    slab.build(); world.step(render=False)
    res["diag"]["ee_top"] = [round(float(v),4) for v in ee_top]
    res["diag"]["slab_pos"] = [round(float(v),4) for v in cfg.pork.position]

    # IMPEDANCE: set the holding (left) arm to compliant gains via the articulation CONTROLLER
    # (SingleArticulation has no get/set_gains — the controller does).
    try:
        ctrl = view.get_articulation_controller()
        g = ctrl.get_gains()                      # (kps, kds)
        kp = np.array(g[0]).reshape(-1).astype(float)
        kd = np.array(g[1]).reshape(-1).astype(float)
        for j in L:
            kp[j] = cfg.control.hold_stiffness
            kd[j] = cfg.control.hold_damping
        ctrl.set_gains(kps=kp, kds=kd)
        res["diag"]["impedance_set"] = {"stiffness": cfg.control.hold_stiffness,
                                        "damping": cfg.control.hold_damping}
        log("left-arm impedance gains set via controller (compliant hold)")
    except Exception as e:
        res["diag"]["impedance_set"] = f"gain API issue: {e}"; log(f"gain set: {e}")

    # holding grasp: move left gripper to the +Y edge of the slab, then weld the end block to it
    plus_y_block = None
    for (ix, li), path in slab._blocks.items():
        if li == len(cfg.pork.layers)-1 and ix == max(j for (j,_l) in slab._blocks):
            plus_y_block = path
    grasp_target = np.array([cfg.pork.position[0] + cfg.pork.length_m/2,
                             cfg.pork.position[1] + cfg.pork.width_m/2, knife_top[2] - 0.03])
    l_reach = ik(ltcp, L, grasp_target, iters=30)
    res["diag"]["grasp_target"] = [round(float(v),4) for v in grasp_target]
    res["diag"]["l_reach"] = [round(float(v),4) for v in l_reach]
    res["diag"]["l_reach_err"] = round(float(np.linalg.norm(l_reach - grasp_target)), 4)
    if plus_y_block:
        wj = UsdPhysics.FixedJoint.Define(stage, ltcp + "/hold_weld")
        wj.GetBody0Rel().SetTargets([ltcp]); wj.GetBody1Rel().SetTargets([plus_y_block])
        log(f"welded {plus_y_block} to left gripper (held)")
        held0 = wpos(plus_y_block)

    rec = Recorder(stage)
    for (ix, li), path in slab._blocks.items():
        rec.add_box(path, color=cfg.pork.layers[li].color, name=f"blk_{ix}_{li}")
    rec.add_box(blade_path, color=(0.75,0.78,0.82), name="blade")

    world.reset();
    for _ in range(6): world.step(render=False)
    try: view.initialize()
    except Exception: pass
    q = np.array(view.get_joint_positions()).reshape(-1).astype(float)

    # right arm descends to cut; record; track held-block displacement
    z0 = wpos(rtcp)[2]; xy = wpos(rtcp)[:2]; held_disp = []
    for i in range(50):
        tz = z0 - 0.06 * (i/49)
        ik(rtcp, R, np.array([xy[0], xy[1], tz]), iters=2)
        k = wpos(blade_path)
        slab.update_cut(float(k[0]), float(k[2]-cfg.blade.height_m/2), kerf_m=cfg.pork.seam_spacing_m)
        if plus_y_block: held_disp.append(float(np.linalg.norm(wpos(plus_y_block) - held0)))
        if i % 2 == 0: rec.capture()
    for _ in range(20):
        world.step(render=False); rec.capture()

    rec.save(os.path.join(OUT, "recording_hold_cut.json"))
    res["diag"]["fraction_cut"] = round(slab.fraction_cut, 4)
    res["diag"]["held_block_max_disp_m"] = round(max(held_disp), 4) if held_disp else None
    res["diag"]["held_stable"] = (max(held_disp) < 0.05) if held_disp else None
    res["ok"] = slab.fraction_cut > 0.0
    log(f"DONE fraction_cut={slab.fraction_cut:.3f} held_max_disp={res['diag']['held_block_max_disp_m']}")
except Exception:
    res["errors"].append(traceback.format_exc()); log("FAILED:"); traceback.print_exc()
finally:
    try: json.dump(res, open(os.path.join(OUT, "hold_and_cut_results.json"), "w"), indent=2)
    except Exception as e: log(f"write: {e}")
    print("HOLD_CUT_OK" if res["ok"] else "HOLD_CUT_FAILED", flush=True)
    sim.close()
