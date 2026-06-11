"""Isaac Sim API compatibility shim.

Isaac Sim renamed its Python API from ``omni.isaac.core.*`` (≤4.x) to ``isaacsim.core.*``
(5.x/6.x). This module resolves the symbols we use from whichever namespace is installed, so the
rest of the codebase imports from here and we adjust *one* file when the API moves again.

These imports only work inside a running Isaac Sim app. On a plain machine the symbols are ``None``
so pure-logic modules (config, material) and unit tests still import.

If something fails to resolve on your SCC build, fix it here and note it in experiments/ENV.md.
"""
from __future__ import annotations

ISAAC_AVAILABLE = True

try:
    # ---- 5.x / 6.x namespace (preferred) ------------------------------------------------
    try:
        from isaacsim.core.api import World  # type: ignore
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, GroundPlane  # type: ignore
    except Exception:
        from isaacsim.core import World  # type: ignore
        from isaacsim.core.objects import DynamicCuboid, FixedCuboid, GroundPlane  # type: ignore

    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # type: ignore
    from isaacsim.core.utils.prims import create_prim, get_prim_at_path  # type: ignore
    from isaacsim.core.utils.types import ArticulationAction  # type: ignore
    try:
        from isaacsim.core.prims import SingleArticulation as Articulation  # type: ignore
    except Exception:
        from isaacsim.core.prims import Articulation  # type: ignore

except Exception:
    try:
        # ---- legacy ≤4.x namespace ------------------------------------------------------
        from omni.isaac.core import World  # type: ignore
        from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane  # type: ignore
        from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage  # type: ignore
        from omni.isaac.core.utils.prims import create_prim, get_prim_at_path  # type: ignore
        from omni.isaac.core.utils.types import ArticulationAction  # type: ignore
        from omni.isaac.core.articulations import Articulation  # type: ignore
    except Exception:  # pragma: no cover — not inside Isaac Sim
        ISAAC_AVAILABLE = False
        World = DynamicCuboid = FixedCuboid = GroundPlane = None  # type: ignore
        add_reference_to_stage = get_current_stage = None  # type: ignore
        create_prim = get_prim_at_path = ArticulationAction = Articulation = None  # type: ignore


def require_isaac() -> None:
    if not ISAAC_AVAILABLE:
        raise RuntimeError(
            "Isaac Sim API not found. This module must run inside a SimulationApp "
            "(see deploy/scc/run_interactive.sh). On a plain machine only config/material import."
        )
