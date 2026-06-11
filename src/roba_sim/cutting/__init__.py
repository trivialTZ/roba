"""Cutting representation (ADR-001): breakable seams for the demo + material physics.

Lazy exports (PEP 562): ``material`` is pure-Python (importable anywhere); ``breakable_seams``
needs a USD runtime. Importing this package does not pull in either until used.
"""
from importlib import import_module

_EXPORTS = {
    "BreakableSlab": "breakable_seams",
    "SeamJoint": "breakable_seams",
    "seam_break_force_n": "material",
    "sharpness_factor": "material",
    "layer_at_height": "material",
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        return getattr(import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
