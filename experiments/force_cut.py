"""Force-based validation of the toughness->break-force model in PhysX (Aim 1.2/2.2 physics depth).

The geometric demo (headless_cut.py) breaks seams by blade position. THIS test breaks them by real
*force*: for each tissue layer, a dynamic block is held to a static anchor by a breakable fixed joint
whose break-force comes from material.seam_break_force_n (skin ~1338 N >> fat ~249 N > lean ~37 N). We
apply a linearly ramping pull and record the force at which PhysX severs the joint. Confirms (a) PhysX
honors the per-layer break-force, and (b) skin >> fat > lean emerges dynamically.

Uses the batched RigidPrim view (isaacsim.core.prims) for force + pose, built from raw pxr prims.
Headless (no RTX). Run via deploy/scc/job_headless.qsub. Output -> $ROBA_OUT.
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
results = {"ok": False, "layers": {}, "summary": {}, "errors": [], "diag": {}}

def log(s): print(f"[force_cut] {s}", flush=True)

try:
    import numpy as np
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import FAT, LEAN, SKIN, default_config
    from roba_sim.cutting.material import seam_break_force_n
    from pxr import Gf, UsdGeom, UsdPhysics

    cfg = default_config()
    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt,
                     stage_units_in_meters=1.0)
    stage = ic.get_current_stage()

    def make_cube(path, pos, size, dynamic, mass=0.1):
        c = UsdGeom.Cube.Define(stage, path)
        c.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(c)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3f(size, size, size))
        prim = c.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        if dynamic:
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(mass)
        return path

    layers = [LEAN, FAT, SKIN]
    configured = {}
    for i, layer in enumerate(layers):
        y = i * 0.4
        make_cube(f"/World/anchor_{layer.name}", (0.0, y, 0.5), 0.05, dynamic=False)
        make_cube(f"/World/block_{layer.name}", (0.06, y, 0.5), 0.05, dynamic=True)
        fbreak = seam_break_force_n(layer, cfg.pork.width_m, cfg.control, cfg.blade)
        configured[layer.name] = fbreak
        j = UsdPhysics.FixedJoint.Define(stage, f"/World/joint_{layer.name}")
        j.GetBody0Rel().SetTargets([f"/World/anchor_{layer.name}"])
        j.GetBody1Rel().SetTargets([f"/World/block_{layer.name}"])
        j.CreateBreakForceAttr(float(fbreak))
        j.CreateBreakTorqueAttr(1.0e9)
        log(f"{layer.name}: configured break-force = {fbreak:.1f} N")

    world.reset()

    # Batched rigid view over the three blocks.
    from isaacsim.core.prims import RigidPrim
    view = RigidPrim(prim_paths_expr="/World/block_.*")
    try:
        view.initialize()
    except Exception:
        pass
    order = [p.split("/")[-1].replace("block_", "") for p in view.prim_paths]
    results["diag"]["view_order"] = order
    results["diag"]["force_methods"] = [m for m in dir(view) if "force" in m.lower()]
    log(f"view order: {order}; force methods: {results['diag']['force_methods']}")
    N = len(order)
    x0 = np.array(view.get_world_poses()[0]).reshape(N, -1)[:, 0].copy()

    RAMP, MAX_STEPS, SEP = 5.0, 400, 0.02
    measured = {}
    for step in range(MAX_STEPS):
        F = RAMP * step
        forces = np.zeros((N, 3), dtype=float)
        for k in range(N):
            if order[k] not in measured:
                forces[k, 0] = F
        try:
            view.apply_forces(forces, is_global=True)
        except TypeError:
            view.apply_forces(forces)
        world.step(render=False)
        pos = np.array(view.get_world_poses()[0]).reshape(N, -1)[:, 0]
        for k in range(N):
            name = order[k]
            if name not in measured and abs(float(pos[k]) - float(x0[k])) > SEP:
                measured[name] = {"measured_break_force_N": round(F, 1),
                                  "configured_break_force_N": round(configured[name], 1),
                                  "break_step": step}
                log(f"{name} SEVERED at F={F:.1f} N (configured {configured[name]:.1f} N)")
        if len(measured) == N:
            break

    results["layers"] = measured
    sorted_order = sorted(measured, key=lambda k: measured[k]["measured_break_force_N"])
    results["summary"] = {
        "break_order_low_to_high": sorted_order,
        "ordering_correct": sorted_order == ["lean", "fat", "skin"],
        "ramp_N_per_step": RAMP,
        "max_relative_error": max(
            abs(measured[k]["measured_break_force_N"] - measured[k]["configured_break_force_N"])
            / measured[k]["configured_break_force_N"] for k in measured) if measured else None,
    }
    results["ok"] = len(measured) == N
    log(f"order low->high: {sorted_order} (expect lean,fat,skin)")

except Exception:
    results["errors"].append(traceback.format_exc())
    log("FAILED:"); traceback.print_exc()
finally:
    try:
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "force_cut_results.json"), "w") as f:
            json.dump(results, f, indent=2)
        log(f"wrote {os.path.join(OUT, 'force_cut_results.json')}")
    except Exception as e:
        log(f"could not write results: {e}")
    print("FORCE_CUT_OK" if results["ok"] else "FORCE_CUT_FAILED", flush=True)
    sim.close()
