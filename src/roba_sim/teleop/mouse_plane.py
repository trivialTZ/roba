"""Mouse cursor → 2-variable cutting-plane target (Aim 1.4).

Per docs/FEASIBILITY.md Pillar 6: the carb input API has no mouse *events*, so we poll the
normalized mouse coordinates each frame and map them to a rectangle on the cutting plane (X along
the cut, Z height). An exponential moving average smooths the raw cursor so the downstream IK
doesn't jitter. Only 2 task-space DOF (X, Z) are commanded; Y is fixed (the plane) and the blade
orientation is fixed (down) — that's the "2D, two-variable" reduction the spec asks for.
"""
from __future__ import annotations

from typing import Optional, Tuple

from ..config import TeleopConfig

try:
    import carb.input
    import omni.appwindow
except Exception:  # pragma: no cover — not inside Isaac Sim
    carb = None  # type: ignore
    omni = None  # type: ignore


class MousePlaneTeleop:
    def __init__(self, cfg: TeleopConfig):
        self.cfg = cfg
        self._ema: Optional[Tuple[float, float]] = None
        self._mouse = None
        self._input = None
        if carb is not None:
            try:
                appwindow = omni.appwindow.get_default_app_window()
                self._mouse = appwindow.get_mouse()
                self._input = carb.input.acquire_input_interface()
            except Exception as exc:  # pragma: no cover
                print(f"[mouse_plane] could not acquire mouse interface: {exc}")

    def _raw_normalized(self) -> Tuple[float, float]:
        """Return mouse position in [0,1]×[0,1]; (0.5, 0.5) if unavailable."""
        if self._input is None or self._mouse is None:
            return 0.5, 0.5
        try:
            c = self._input.get_mouse_coords_normalized(self._mouse)
            return float(c[0]), float(c[1])
        except Exception:
            try:  # fallback: pixels ÷ window size
                px = self._input.get_mouse_coords_pixel(self._mouse)
                aw = omni.appwindow.get_default_app_window()
                return float(px[0]) / max(1, aw.get_width()), float(px[1]) / max(1, aw.get_height())
            except Exception:
                return 0.5, 0.5

    def read_plane_target(self) -> Tuple[float, float]:
        """Return the smoothed (x, z) world target on the cutting plane."""
        mx, my = self._raw_normalized()
        if self.cfg.invert_y:
            my = 1.0 - my
        x = _lerp(self.cfg.x_range_m, mx)
        z = _lerp(self.cfg.z_range_m, my)
        if self._ema is None:
            self._ema = (x, z)
        else:
            a = self.cfg.smoothing_alpha
            self._ema = (a * x + (1 - a) * self._ema[0], a * z + (1 - a) * self._ema[1])
        return self._ema

    def reset(self) -> None:
        self._ema = None


def _lerp(rng: Tuple[float, float], t: float) -> float:
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    return rng[0] + (rng[1] - rng[0]) * t
