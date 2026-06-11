"""Energy-based mapping from tissue fracture mechanics to breakable-joint thresholds.

This is the physics that makes the pre-scored-seam cutting (ADR-001 option b) behave like real
tissue: each seam between two sub-blocks is a breakable PhysX joint whose break-force is derived
from the layer's fracture toughness, the cut width, blade sharpness, friction, and the current
slice-push ratio. Pure functions, no Isaac Sim import — unit-testable on any machine.

Model (Atkins / Lucas energy approach, see docs/MATERIAL_MODEL.md):

    Cutting force per unit width ≈ fracture toughness  (units: J/m² · m = N/m · m = N)

so for a seam of width ``w`` the fracture term is ``Jc · w``. Slicing (a draw motion) lowers the
effective toughness; we model the well-reported ~40% reduction linearly in the slice-push ratio.
Friction adds a term proportional to the normal load and the blade-tissue μ. A sharper edge
(smaller edge radius) lowers the force via a bounded sharpness factor.
"""
from __future__ import annotations

from ..config import BladeConfig, ControlConfig, LayerParams

# Maximum fractional reduction in cutting force from a full draw (slice_push_ratio = 1).
SLICE_FORCE_REDUCTION = 0.40
# Reference edge radius (m) at which the sharpness factor is 1.0; sharper → <1, blunter → >1.
REF_EDGE_RADIUS_M = 1.0e-4


def sharpness_factor(edge_radius_m: float) -> float:
    """Bounded multiplier on cutting force from blade sharpness.

    A very sharp edge approaches ~0.625× of the reference; a very blunt edge saturates at 2.5×.
    Monotonic in edge radius, smooth, and clamped so the demo never produces silly numbers.
    """
    ratio = max(edge_radius_m, 1e-6) / REF_EDGE_RADIUS_M
    # 0.5 .. 2.0 over ~two decades of radius, centered at the reference radius.
    factor = 0.5 + 0.5 * min(max(ratio, 0.25), 4.0)
    return factor


def seam_break_force_n(
    layer: LayerParams,
    seam_width_m: float,
    control: ControlConfig,
    blade: BladeConfig,
    normal_load_n: float = 0.0,
) -> float:
    """Break-force threshold (N) for a seam joint in ``layer``.

    Args:
        layer: the tissue layer the seam belongs to.
        seam_width_m: cut width of this seam (the across-cut dimension of the sub-block, Y).
        control: provides the current slice_push_ratio.
        blade: provides the edge radius (sharpness).
        normal_load_n: optional estimate of the normal contact load, for the friction term.

    Returns:
        Force threshold above which the joint snaps. Ordering skin ≫ fat > lean is preserved.
    """
    # Fracture term: toughness × width.
    fracture_n = layer.fracture_toughness_j_m2 * seam_width_m

    # Slicing reduces effective toughness (draw cut). ratio in [0, 1].
    slice_reduction = 1.0 - SLICE_FORCE_REDUCTION * _clamp01(control.slice_push_ratio)

    # Sharpness scales the fracture term.
    sharp = sharpness_factor(blade.edge_radius_m)

    # Friction term: μ × normal load (small relative to fracture for sharp blades, but real).
    friction_n = layer.friction_mu * max(normal_load_n, 0.0)

    return fracture_n * slice_reduction * sharp + friction_n


def layer_at_height(layers, z_local_m: float) -> LayerParams:
    """Return the layer occupying local height ``z_local_m`` (0 = bottom of the slab).

    ``layers`` is bottom→top (as in PorkBellyConfig.layers). Used to tag each seam/sub-block.
    """
    z = 0.0
    for layer in layers:
        z += layer.thickness_m
        if z_local_m <= z:
            return layer
    return layers[-1]


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x
