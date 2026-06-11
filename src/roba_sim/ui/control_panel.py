"""omni.ui control panel (Aim 1.5).

Sliders mutate the live ``RobaConfig`` (the app reads it every step, so changes take effect
immediately); buttons drive start/pause/reset; a combo box switches between mouse teleop and the
two autonomous tasks. Built on omni.ui per docs/FEASIBILITY.md Pillar 7.
"""
from __future__ import annotations

from typing import Callable, Dict

from ..config import RobaConfig

try:
    import omni.ui as ui
except Exception:  # pragma: no cover — not inside Isaac Sim
    ui = None  # type: ignore

MODES = ["Manual (mouse)", "Skive skin (2.4a)", "Vertical slice (2.4b)"]


class RobaControlPanel:
    def __init__(self, cfg: RobaConfig, callbacks: Dict[str, Callable]):
        if ui is None:
            raise RuntimeError("omni.ui not available — run inside Isaac Sim GUI/livestream")
        self.cfg = cfg
        self.cb = callbacks  # keys: start, pause, reset, set_mode(int)
        self._window = ui.Window("roba — meat cutting", width=380, height=460)
        self._status_label = None
        self._build()

    def _build(self) -> None:
        c = self.cfg.control
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                ui.Label("Cutting parameters", style={"font_size": 16})
                self._slider("Slice-push ratio", 0.0, 1.0, c.slice_push_ratio,
                             lambda v: setattr(c, "slice_push_ratio", v))
                self._slider("Blade edge radius (mm)", 0.02, 1.0,
                             self.cfg.blade.edge_radius_m * 1000.0,
                             lambda v: setattr(self.cfg.blade, "edge_radius_m", v / 1000.0))
                self._slider("Max press force (N)", 20.0, 300.0, c.max_press_force_n,
                             lambda v: setattr(c, "max_press_force_n", v))

                ui.Spacer(height=6)
                ui.Label("Holding arm impedance", style={"font_size": 16})
                self._slider("Hold stiffness", 200.0, 12000.0, c.hold_stiffness,
                             lambda v: (setattr(c, "hold_stiffness", v),
                                        self.cb.get("set_stiffness", lambda _x: None)(v)))
                self._slider("Hold damping", 20.0, 1200.0, c.hold_damping,
                             lambda v: setattr(c, "hold_damping", v))

                ui.Spacer(height=6)
                ui.Label("Autonomous task params", style={"font_size": 16})
                self._slider("Skive depth (mm)", 0.5, 10.0, c.skive_depth_m * 1000.0,
                             lambda v: setattr(c, "skive_depth_m", v / 1000.0))
                self._slider("Slice spacing (mm)", 4.0, 30.0, c.slice_step_m * 1000.0,
                             lambda v: setattr(c, "slice_step_m", v / 1000.0))

                ui.Spacer(height=8)
                ui.Label("Mode")
                combo = ui.ComboBox(0, *MODES)
                combo.model.add_item_changed_fn(
                    lambda m, _i: self.cb.get("set_mode", lambda _x: None)(
                        m.get_item_value_model().get_value_as_int()))

                ui.Spacer(height=8)
                with ui.HStack(height=32, spacing=6):
                    ui.Button("Start", clicked_fn=self.cb.get("start", lambda: None))
                    ui.Button("Pause", clicked_fn=self.cb.get("pause", lambda: None))
                    ui.Button("Reset", clicked_fn=self.cb.get("reset", lambda: None))

                ui.Spacer(height=6)
                self._status_label = ui.Label("idle", style={"color": 0xFF88FF88})

    def _slider(self, label, lo, hi, init, setter) -> None:
        with ui.HStack(height=24):
            ui.Label(label, width=170)
            model = ui.SimpleFloatModel(float(init))
            ui.FloatSlider(model=model, min=float(lo), max=float(hi))
            model.add_value_changed_fn(lambda m: setter(m.get_value_as_float()))

    def set_status(self, text: str) -> None:
        if self._status_label is not None:
            self._status_label.text = text
