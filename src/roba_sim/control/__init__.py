"""Control (Aim 1.3 / 2.4): impedance holding + cutting motion generation.

Lazy exports: ``cutting_controller`` is pure-Python (trajectory math, testable anywhere);
``impedance_hold`` needs numpy/Isaac. Not imported until accessed.
"""
from importlib import import_module

_EXPORTS = {
    "ImpedanceHoldController": "impedance_hold",
    "CuttingController": "cutting_controller",
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        return getattr(import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
