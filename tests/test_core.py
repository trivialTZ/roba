"""Unit tests for the GPU-free core (material physics, config, trajectories).

These run on any machine (no Isaac Sim) — the version-agnostic heart of Aim 1.2/2.4.
Run:  python -m pytest tests/ -q   (or: python tests/test_core.py)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from roba_sim.config import FAT, LEAN, SKIN, default_config  # noqa: E402
from roba_sim.control.cutting_controller import CuttingController  # noqa: E402
from roba_sim.cutting.material import (  # noqa: E402
    layer_at_height, seam_break_force_n, sharpness_factor,
)


def test_break_force_ordering_skin_strongest():
    cfg = default_config()
    w = cfg.pork.width_m
    fs = seam_break_force_n(SKIN, w, cfg.control, cfg.blade)
    ff = seam_break_force_n(FAT, w, cfg.control, cfg.blade)
    fl = seam_break_force_n(LEAN, w, cfg.control, cfg.blade)
    assert fs > ff > fl, (fs, ff, fl)  # skin ≫ fat > lean


def test_slicing_lowers_break_force():
    cfg = default_config()
    cfg.control.slice_push_ratio = 0.0
    f_press = seam_break_force_n(LEAN, cfg.pork.width_m, cfg.control, cfg.blade)
    cfg.control.slice_push_ratio = 1.0
    f_slice = seam_break_force_n(LEAN, cfg.pork.width_m, cfg.control, cfg.blade)
    assert f_slice < f_press
    assert abs((f_press - f_slice) / f_press - 0.40) < 1e-6  # ~40% reduction


def test_sharpness_monotonic_and_bounded():
    sharp = sharpness_factor(1e-5)   # very sharp
    ref = sharpness_factor(1e-4)     # reference
    blunt = sharpness_factor(1e-3)   # blunt
    assert sharp < ref < blunt
    assert 0.6 < sharp and blunt <= 2.5     # factor range is [0.625, 2.5]


def test_layer_at_height():
    layers = [LEAN, FAT, SKIN]  # bottom→top, thicknesses 0.020, 0.012, 0.004
    assert layer_at_height(layers, 0.001).name == "lean"
    assert layer_at_height(layers, 0.025).name == "fat"
    assert layer_at_height(layers, 0.035).name == "skin"


def test_skive_trajectory_constant_depth():
    cfg = default_config()
    cc = CuttingController(cfg.control, cfg.pork, cfg.blade)
    poses = list(cc.skive_trajectory(n_steps=50))
    zs = [p[0][2] for p in poses]
    assert max(zs) - min(zs) < 1e-9               # constant depth
    top = cfg.pork.position[2] + cfg.pork.total_thickness_m
    assert abs(zs[0] - (top - cfg.control.skive_depth_m)) < 1e-9
    xs = [p[0][0] for p in poses]
    assert xs[0] < xs[-1]                          # advances along +X


def test_slice_trajectory_presses_down():
    cfg = default_config()
    cc = CuttingController(cfg.control, cfg.pork, cfg.blade)
    poses = list(cc.slice_trajectory(dwell_steps=20))
    assert len(poses) > 0
    # within one station the z should descend toward the floor
    z_first, z_last = poses[0][0][2], poses[19][0][2]
    assert z_first > z_last
    assert min(p[0][2] for p in poses) >= cfg.control.cut_plane_z_floor_m - 1e-9


def test_force_bound_holds_when_force_exceeds_cap():
    cfg = default_config()
    cc = CuttingController(cfg.control, cfg.pork, cfg.blade)
    cc.target_from_plane(0.0, 0.10, 0.0)           # establish last_z
    z_before = cc._last_z
    pos, _ = cc.target_from_plane(0.0, 0.02, 0.1, measured_force_n=cfg.control.max_press_force_n + 10)
    assert pos[2] == z_before                       # blade held, did not descend


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
        passed += 1
    print(f"\n{passed}/{len(fns)} tests passed")
