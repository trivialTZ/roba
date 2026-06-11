"""Teleoperation (Aim 1.4): mouse → cutting-plane target → constrained IK. Lazy exports."""
from importlib import import_module

_EXPORTS = {"MousePlaneTeleop": "mouse_plane", "PlaneConstrainedIK": "plane_ik"}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        return getattr(import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
