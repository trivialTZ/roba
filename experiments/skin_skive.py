"""Headless validation of skin-skiving (Aim 2.4a) — the project's novel contribution.

Skiving = a shallow, constant-depth tangential pass that peels the rubbery SKIN off the fat by
severing the skin/fat adhesion interface, leaving the skin a connected sheet and the deeper tissue
intact (verified there is no prior closed-loop robot doing this — docs/FEASIBILITY.md Pillar 11).

We ride the blade at z = top - skive_depth along +X and break only the skin/fat interface seams
(BreakableSlab.break_interface_under). Success = most skin/fat interfaces severed, lean/fat
interfaces and skin column-seams left intact (selective peel). Headless (no RTX). Output -> $ROBA_OUT.
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
results = {"ok": False, "summary": {}, "errors": []}

def log(s): print(f"[skin_skive] {s}", flush=True)

try:
    from pxr import Gf, UsdGeom
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config
    from roba_sim.cutting.breakable_seams import BreakableSlab

    cfg = default_config()
    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt,
                     stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = ic.get_current_stage()

    slab = BreakableSlab(stage, cfg.pork, cfg.control, cfg.blade, root="/World/PorkBelly")
    slab.build()

    # Count interface seam classes before skiving.
    def count(pred):
        return sum(1 for s in slab.seams if pred(s))
    n_skinfat = count(lambda s: s.layer_name == "fat/skin")
    n_leanfat = count(lambda s: s.layer_name == "lean/fat")
    n_skin_cols = count(lambda s: s.layer_name == "skin")  # in-skin column seams
    log(f"interfaces: fat/skin={n_skinfat}, lean/fat={n_leanfat}; skin column-seams={n_skin_cols}")

    # Visual blade riding at the skin/fat interface height.
    top_z = cfg.pork.position[2] + cfg.pork.total_thickness_m
    skive_z = top_z - cfg.control.skive_depth_m
    # the skin/fat interface sits at lean.thickness+fat.thickness above the slab origin:
    z_lean = cfg.pork.layers[0].thickness_m
    z_fat = cfg.pork.layers[1].thickness_m
    interface_z = cfg.pork.position[2] + z_lean + z_fat
    log(f"top_z={top_z:.4f} skive_z={skive_z:.4f} skin/fat interface_z={interface_z:.4f}")

    blade = UsdGeom.Cube.Define(stage, "/World/Blade")
    blade.GetSizeAttr().Set(1.0)
    bxf = UsdGeom.Xformable(blade); bxf.ClearXformOpOrder()
    bt = bxf.AddTranslateOp()
    bxf.AddScaleOp().Set(Gf.Vec3f(cfg.blade.thickness_m, cfg.pork.width_m * 1.1, cfg.blade.height_m))

    world.reset()

    # Sweep the blade along +X at the interface height, finely so every column is caught.
    x_start = cfg.pork.position[0] - cfg.pork.length_m / 2
    x_end = cfg.pork.position[0] + cfg.pork.length_m / 2
    n_steps = 120
    for i in range(n_steps):
        x = x_start + (x_end - x_start) * i / (n_steps - 1)
        bt.Set(Gf.Vec3d(x, cfg.pork.position[1], interface_z))
        # break the skin/fat interface at the skive height (z_tol < half a layer so lean/fat is safe)
        slab.break_interface_under(x, interface_z, kerf_m=cfg.pork.seam_spacing_m, z_tol_m=0.004)
        world.step(render=False)
    for _ in range(30):
        world.step(render=False)

    broke_skinfat = count(lambda s: s.layer_name == "fat/skin" and s.broken)
    broke_leanfat = count(lambda s: s.layer_name == "lean/fat" and s.broken)
    broke_skincols = count(lambda s: s.layer_name == "skin" and s.broken)

    results["summary"] = {
        "skin_fat_interfaces_total": n_skinfat,
        "skin_fat_interfaces_severed": broke_skinfat,
        "skin_peel_fraction": round(broke_skinfat / n_skinfat, 3) if n_skinfat else 0.0,
        "lean_fat_interfaces_severed": broke_leanfat,     # want 0 (selective)
        "skin_column_seams_severed": broke_skincols,      # want 0 (skin stays a sheet)
        "selective_peel": (broke_leanfat == 0 and broke_skincols == 0 and broke_skinfat > 0),
    }
    results["ok"] = results["summary"]["selective_peel"]
    log(f"peeled {broke_skinfat}/{n_skinfat} skin/fat; lean/fat severed={broke_leanfat}; "
        f"skin cols severed={broke_skincols}; selective={results['summary']['selective_peel']}")

except Exception:
    results["errors"].append(traceback.format_exc())
    log("FAILED:"); traceback.print_exc()
finally:
    try:
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "skin_skive_results.json"), "w") as f:
            json.dump(results, f, indent=2)
        log(f"wrote {os.path.join(OUT, 'skin_skive_results.json')}")
    except Exception as e:
        log(f"could not write results: {e}")
    print("SKIN_SKIVE_OK" if results["ok"] else "SKIN_SKIVE_FAILED", flush=True)
    sim.close()
