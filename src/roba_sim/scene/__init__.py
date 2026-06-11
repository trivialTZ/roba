"""Scene assembly: dual arms, knife end-effector, layered pork belly, world.

Lazy exports — everything here needs a USD/Isaac runtime, so nothing imports at package load.
"""
from importlib import import_module

_EXPORTS = {"RobaWorld": "world"}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        return getattr(import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
