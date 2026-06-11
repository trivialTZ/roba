"""Headless dual-arm scene assembly (Aim 1.3) — loading-validation pass.

Step 1 of dual-arm integration: prove the scene assembles headless and learn the live API/link names
before wiring IK-driven cutting. Loads the OpenArm BIMANUAL USD (left arm = holder, right arm =
cutter; loads cleanly, no URDF import needed — ADR-002 fallback), builds the 3-layer slab between the
arms, attaches a knife to the right arm's TCP, and ALSO test-imports the reBot B601 URDF separately
(satisfies "import two models"; non-fatal if the importer API differs). Prints both arms' dof_names.

Headless (no RTX). Output -> $ROBA_OUT.
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
results = {"ok": False, "diag": {}, "errors": []}

def log(s): print(f"[dual_arm] {s}", flush=True)

try:
    from pxr import Gf, UsdGeom, UsdPhysics
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config

    cfg = default_config()
    asset_root = os.environ.get("ROBA_ASSET_ROOT", "/workspace/assets/robots")
    bimanual = (f"{asset_root}/openarm_isaac_lab/source/openarm/openarm/tasks/"
                f"manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd")

    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt,
                     stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = ic.get_current_stage()

    # --- bimanual OpenArm (the working dual-arm platform) ---
    log(f"loading bimanual USD exists={os.path.exists(bimanual)}")
    ic.add_reference_to_stage(usd_path=bimanual, prim_path="/World/OpenArmBi")

    # --- 3-layer slab between the arms ---
    from roba_sim.cutting.breakable_seams import BreakableSlab
    slab = BreakableSlab(stage, cfg.pork, cfg.control, cfg.blade, root="/World/PorkBelly")
    slab.build()
    log(f"slab built: {len(slab.seams)} seams")

    world.reset()
    for _ in range(3):
        world.step(render=False)

    # --- introspect the bimanual articulation(s) ---
    try:
        art = (ic.Articulation(prim_paths_expr="/World/OpenArmBi")
               if "prim_paths_expr" in ic.Articulation.__init__.__code__.co_varnames
               else ic.Articulation(prim_path="/World/OpenArmBi"))
        world.reset()
        for _ in range(2):
            world.step(render=False)
        dof = list(getattr(art, "dof_names", []) or [])
        results["diag"]["bimanual_dof_names"] = dof
        results["diag"]["bimanual_num_dof"] = len(dof)
        log(f"bimanual dof ({len(dof)}): {dof}")
    except Exception:
        results["errors"].append("bimanual articulation: " + traceback.format_exc())
        log("bimanual articulation introspection failed (see errors)")

    # --- attach a knife to the right-arm TCP (find the tcp prim by name) ---
    tcp = None
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if p.startswith("/World/OpenArmBi") and ("right" in p) and p.endswith("ee_tcp"):
            tcp = p; break
    results["diag"]["right_tcp_prim"] = tcp
    if tcp:
        blade_path = tcp + "/knife"
        c = UsdGeom.Cube.Define(stage, blade_path); c.GetSizeAttr().Set(1.0)
        bxf = UsdGeom.Xformable(c); bxf.ClearXformOpOrder()
        bxf.AddScaleOp().Set(Gf.Vec3f(cfg.blade.length_m, cfg.blade.thickness_m, cfg.blade.height_m))
        prim = c.GetPrim(); UsdPhysics.RigidBodyAPI.Apply(prim); UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(cfg.blade.mass_kg)
        j = UsdPhysics.FixedJoint.Define(stage, blade_path + "_mount")
        j.GetBody0Rel().SetTargets([tcp]); j.GetBody1Rel().SetTargets([blade_path])
        log(f"knife attached at {blade_path}")
    else:
        log("right-arm TCP prim not found by name — will inspect prim list")
        results["diag"]["openarmbi_prims_sample"] = [
            str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()).startswith("/World/OpenArmBi")
        ][:60]

    # --- test-import reBot URDF separately (non-fatal) ---
    try:
        from roba_sim.scene.arms import import_urdf
        import_urdf(cfg.holding_arm.urdf_path, "/World/reBot")
        results["diag"]["rebot_import"] = "ok"
        log("reBot URDF imported")
    except Exception as e:
        results["diag"]["rebot_import"] = f"failed: {e}"
        log(f"reBot URDF import failed (non-fatal): {e}")

    results["ok"] = results["diag"].get("bimanual_num_dof", 0) > 0
    log(f"DONE ok={results['ok']}")

except Exception:
    results["errors"].append(traceback.format_exc())
    log("FAILED:"); traceback.print_exc()
finally:
    try:
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "dual_arm_results.json"), "w") as f:
            json.dump(results, f, indent=2)
        log(f"wrote {os.path.join(OUT, 'dual_arm_results.json')}")
    except Exception as e:
        log(f"could not write results: {e}")
    print("DUAL_ARM_OK" if results["ok"] else "DUAL_ARM_FAILED", flush=True)
    sim.close()
