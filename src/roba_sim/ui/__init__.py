"""User interface (Aim 1.5): omni.ui control panel. Lazy export (needs Isaac GUI)."""
from importlib import import_module

_EXPORTS = {"RobaControlPanel": "control_panel"}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        return getattr(import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
