"""Headless cut WITH per-frame recording for driver-agnostic playback (Tier-1 visualization).

Reuses the validated vertical-slice mechanic (headless_cut.py): a kinematic blade descends through
the 3-layer breakable-seam slab at several X stations. Records every slab sub-block + the blade each
frame via roba_sim.recording.Recorder, so the cut (including pieces separating) can be rendered later
with matplotlib — no Isaac renderer / NVIDIA-driver RTX needed. Headless. Output -> $ROBA_OUT.
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
status = {"ok": False, "errors": []}

def log(s): print(f"[record_cut] {s}", flush=True)

try:
    from pxr import Gf, UsdGeom
    sys.path.insert(0, "/workspace/roba/src")
    from roba_sim import _isaac_compat as ic
    from roba_sim.config import default_config
    from roba_sim.cutting.breakable_seams import BreakableSlab
    from roba_sim.recording import Recorder

    cfg = default_config()
    world = ic.World(physics_dt=cfg.sim.physics_dt, rendering_dt=cfg.sim.rendering_dt,
                     stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = ic.get_current_stage()

    slab = BreakableSlab(stage, cfg.pork, cfg.control, cfg.blade, root="/World/PorkBelly")
    slab.build()

    # kinematic blade
    blade_path = "/World/Blade"
    cube = UsdGeom.Cube.Define(stage, blade_path); cube.GetSizeAttr().Set(1.0)
    bxf = UsdGeom.Xformable(cube); bxf.ClearXformOpOrder()
    blade_t = bxf.AddTranslateOp()
    bxf.AddScaleOp().Set(Gf.Vec3f(cfg.blade.thickness_m, cfg.pork.width_m * 1.1, cfg.blade.height_m))

    # register boxes to record: every slab sub-block (colored by layer) + the blade
    rec = Recorder(stage)
    for (ix, li), path in slab._blocks.items():
        rec.add_box(path, color=cfg.pork.layers[li].color, name=f"blk_{ix}_{li}")
    rec.add_box(blade_path, color=(0.75, 0.78, 0.82), name="blade")

    world.reset()

    top_z = cfg.pork.position[2] + cfg.pork.total_thickness_m
    floor_z = cfg.control.cut_plane_z_floor_m
    ox = cfg.pork.position[0]
    x_stations = [ox - 0.05, ox, ox + 0.05]
    descend_steps = 40
    REC_EVERY = 2

    s = 0
    for xs in x_stations:
        for d in range(descend_steps):
            frac = d / (descend_steps - 1)
            blade_z = top_z + 0.02 - (top_z + 0.02 - floor_z) * frac
            blade_t.Set(Gf.Vec3d(xs, cfg.pork.position[1], blade_z + cfg.blade.height_m / 2))
            slab.update_cut(xs, blade_z)
            world.step(render=False)
            if s % REC_EVERY == 0:
                rec.capture()
            s += 1
    # let separated pieces settle (and record them falling)
    for d in range(40):
        world.step(render=False)
        if d % REC_EVERY == 0:
            rec.capture()

    rec.save(os.path.join(OUT, "recording_slice.json"))
    status["ok"] = True
    status["n_frames"] = len(rec.frames)
    status["n_boxes"] = len(rec.meta)
    status["fraction_cut"] = round(slab.fraction_cut, 4)
    log(f"recorded {len(rec.frames)} frames x {len(rec.meta)} boxes; fraction_cut={slab.fraction_cut:.3f}")

except Exception:
    status["errors"].append(traceback.format_exc())
    log("FAILED:"); traceback.print_exc()
finally:
    try:
        with open(os.path.join(OUT, "record_cut_status.json"), "w") as f:
            json.dump(status, f, indent=2)
    except Exception as e:
        log(f"status write failed: {e}")
    print("RECORD_CUT_OK" if status["ok"] else "RECORD_CUT_FAILED", flush=True)
    sim.close()
