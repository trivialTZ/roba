"""Headless validation of the breakable-seam cutting (Aim 1.2/2.4) in real PhysX.

Runs WITHOUT RTX rendering (SCC driver 595 blocks it). Builds the 3-layer pork-belly slab from our
own cutting/breakable_seams.py, sweeps a scripted blade down through it at several X stations
(vertical slicing, Aim 2.4b), and records the cut progress + per-layer seam breaks. Loads the
OpenArm USD into the scene too (non-fatal) for completeness. Saves metrics JSON for plotting on the Mac.

Run via deploy/scc/job_headless.qsub (sets the headless Vulkan workaround). Output → $ROBA_OUT.
"""
import json
import os
import sys
import traceback

# 1) SimulationApp FIRST (before any isaacsim/omni/pxr import).
try:
    from isaacsim import SimulationApp
except Exception:
    from omni.isaac.kit import SimulationApp

sim = SimulationApp({"headless": True})  # render=False everywhere below — no RTX needed

OUT = os.environ.get("ROBA_OUT", "/tmp")
results = {"ok": False, "steps": [], "summary": {}, "errors": []}

def log(s):
    print(f"[headless_cut] {s}", flush=True)

try:
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config
    from roba_sim.cutting.breakable_seams import BreakableSlab
    from pxr import Gf, UsdGeom

    cfg = default_config()
    cfg.sim.headless = True

    log("building world + ground")
    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt,
                     stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = ic.get_current_stage()

    log("building 3-layer breakable slab")
    slab = BreakableSlab(stage, cfg.pork, cfg.control, cfg.blade, root="/World/PorkBelly")
    slab.build()
    n_seams = len(slab.seams)
    n_x = slab._n_cols
    log(f"slab built: {n_x} columns x {len(cfg.pork.layers)} layers, {n_seams} seams")

    # Optional: drop the OpenArm USD into the scene (non-fatal) for completeness.
    try:
        ic.add_reference_to_stage(usd_path=cfg.cutting_arm.usd_path, prim_path="/World/OpenArm")
        log("OpenArm USD referenced")
    except Exception as e:
        results["errors"].append(f"openarm load: {e}")
        log(f"OpenArm load skipped: {e}")

    # A simple kinematic visual blade (the cut itself is geometric via slab.update_cut).
    blade_path = "/World/Blade"
    cube = UsdGeom.Cube.Define(stage, blade_path)
    cube.GetSizeAttr().Set(1.0)
    bxf = UsdGeom.Xformable(cube)
    bxf.ClearXformOpOrder()
    blade_t = bxf.AddTranslateOp()
    bxf.AddScaleOp().Set(Gf.Vec3f(cfg.blade.thickness_m, cfg.pork.width_m * 1.1, cfg.blade.height_m))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.75, 0.78, 0.82)])

    world.reset()

    # Vertical slicing sweep: descend the blade at several X stations (Aim 2.4b).
    top_z = cfg.pork.position[2] + cfg.pork.total_thickness_m
    floor_z = cfg.control.cut_plane_z_floor_m
    ox = cfg.pork.position[0]
    x_stations = [ox - 0.05, ox, ox + 0.05]
    descend_steps = 40

    step_i = 0
    for xs in x_stations:
        for d in range(descend_steps):
            frac = d / (descend_steps - 1)
            blade_z = top_z + 0.02 - (top_z + 0.02 - floor_z) * frac
            blade_t.Set(Gf.Vec3d(xs, cfg.pork.position[1], blade_z + cfg.blade.height_m / 2))
            broke = slab.update_cut(xs, blade_z)
            world.step(render=False)
            step_i += 1
            if broke or d == descend_steps - 1:
                results["steps"].append({
                    "step": step_i, "x": round(xs, 4), "blade_z": round(blade_z, 4),
                    "broke_this_step": broke, "fraction_cut": round(slab.fraction_cut, 4),
                })

    # settle a few steps so separated pieces fall under gravity (physics sanity).
    for _ in range(30):
        world.step(render=False)

    broken_by_layer = {}
    for s in slab.seams:
        if s.broken:
            broken_by_layer[s.layer_name] = broken_by_layer.get(s.layer_name, 0) + 1

    results["summary"] = {
        "n_columns": n_x,
        "n_seams_total": n_seams,
        "n_seams_broken": sum(1 for s in slab.seams if s.broken),
        "fraction_cut": round(slab.fraction_cut, 4),
        "broken_by_layer": broken_by_layer,
        "x_stations": x_stations,
        "physics_steps": step_i + 30,
    }
    results["ok"] = True
    log(f"DONE — fraction_cut={slab.fraction_cut:.3f}, broken_by_layer={broken_by_layer}")

except Exception:
    results["errors"].append(traceback.format_exc())
    log("FAILED:")
    traceback.print_exc()
finally:
    try:
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "headless_cut_results.json"), "w") as f:
            json.dump(results, f, indent=2)
        log(f"wrote {os.path.join(OUT, 'headless_cut_results.json')}")
    except Exception as e:
        log(f"could not write results: {e}")
    print("HEADLESS_CUT_OK" if results["ok"] else "HEADLESS_CUT_FAILED", flush=True)
    sim.close()
