"""video2traj — RGB cutting video → kinematic trajectory prior (Aim 2.3, ADR-005).

Outputs trajectories/strategy only; forces are out of scope here (occluded blade, ill-posed).
"""
from .pipeline import TrajectoryPrior, TrajectorySample, VideoTrajectoryPipeline

__all__ = ["VideoTrajectoryPipeline", "TrajectoryPrior", "TrajectorySample"]
