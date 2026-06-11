"""Verify the reBot B601 URDF imports into Isaac Sim 6.0 (Aim 1.1 — second arm model).

Uses the version-fixed import_urdf (scene/arms.py) — Isaac 6.0 dropped the old URDFCreateImportConfig
command in favor of _urdf.ImportConfig(). Confirms the arm articulates: prints dof_names + body_names.
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
def log(s): print(f"[verify_rebot] {s}", flush=True)

try:
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config
    from roba_sim.scene.arms import import_urdf

    cfg = default_config()
    world = ic.World(physics_dt=cfg.sim.physics_dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    log(f"importing reBot URDF: {cfg.holding_arm.urdf_path}")
    import_urdf(cfg.holding_arm.urdf_path, "/World/reBot")
    res["diag"]["import"] = "ok"

    art = (ic.Articulation(prim_paths_expr="/World/reBot")
           if "prim_paths_expr" in ic.Articulation.__init__.__code__.co_varnames
           else ic.Articulation(prim_path="/World/reBot"))
    try: world.scene.add(art)
    except Exception as e: log(f"scene.add: {e}")
    world.reset()
    for _ in range(8): world.step(render=False)
    try: art.initialize()
    except Exception as e: log(f"init: {e}")

    res["diag"]["dof_names"] = list(art.dof_names)
    res["diag"]["num_dof"] = len(art.dof_names)
    try: res["diag"]["body_names"] = list(art.body_names)
    except Exception: pass
    # actuate joint1 a little to confirm it's a live articulation
    import numpy as np
    q = np.array(art.get_joint_positions()).reshape(-1).astype(float)
    res["diag"]["q0_size"] = int(q.size)
    if q.size >= 6:
        q[0] += 0.3
        art.apply_action(ic.ArticulationAction(joint_positions=q))
        for _ in range(20): world.step(render=False)
        q1 = np.array(art.get_joint_positions()).reshape(-1)[0]
        res["diag"]["joint1_moved_to"] = round(float(q1), 3)
    res["ok"] = res["diag"]["num_dof"] >= 6 and q.size == res["diag"]["num_dof"]
    log(f"reBot: {res['diag']['num_dof']} DOF, dof={res['diag']['dof_names']}")
except Exception:
    res["errors"].append(traceback.format_exc()); log("FAILED:"); traceback.print_exc()
finally:
    try:
        os.makedirs(OUT, exist_ok=True)
        json.dump(res, open(os.path.join(OUT, "verify_rebot_results.json"), "w"), indent=2)
    except Exception as e: log(f"write: {e}")
    print("VERIFY_REBOT_OK" if res["ok"] else "VERIFY_REBOT_FAILED", flush=True)
    sim.close()
