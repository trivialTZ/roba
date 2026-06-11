"""roba — robotic meat-cutting simulation in NVIDIA Isaac Sim.

See docs/ARCHITECTURE.md for the binding decisions and docs/FEASIBILITY.md for the verified
constraints (most importantly: Isaac Sim cannot cut deformables natively → breakable seams).
"""
from .config import RobaConfig, default_config

__all__ = ["RobaConfig", "default_config"]
__version__ = "0.1.0"
