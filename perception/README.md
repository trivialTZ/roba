# perception — video understanding → cutting model (Aim 2.3)

Implements **stage 1** of the decoupled pipeline (ADR-005): RGB cutting video → **kinematic
trajectory prior** (knife/hand tracks, motion phases, slice-vs-press strategy). It deliberately
does **not** estimate forces — that is physically unrecoverable from RGB (occluded blade,
ill-posed inverse problem; verified in `docs/FEASIBILITY.md` Pillar 10). Forces come from the
separate **sensor + DiSECt-calibration** stage.

```
video2traj/
  pipeline.py   VideoTrajectoryPipeline: frames → optical flow → tool track → TrajectoryPrior
                (+ plug points for hand pose, tool segmentation, monocular depth)
```

## Usage
```python
from video2traj import VideoTrajectoryPipeline
prior = VideoTrajectoryPipeline(sample_fps=10).track("meat_cut_tutorial.mp4")
prior.to_json("trajectory_prior.json")
print(prior.slice_push_estimate)   # first-order press-vs-slice strategy descriptor
```

## What works now vs. what to wire on SCC
- **Works (model-free):** frame extraction, dense optical flow, motion-saliency tool tracking,
  heuristic phase labels. Enough to validate the plumbing.
- **Plug points (need GPU/weights):** `hand_pose` (MediaPipe/HaMeR), `tool_segmentation` (SAM2 /
  surgical instrument seg), `monocular_depth` (Depth-Anything-V2 — lifts 2D tracks to 3D, resolves
  scale). See `docs/REFERENCES.md` Pillar 10 for sources/datasets (EPIC-KITCHENS, YouCook2,
  Ego-Exo4D for food; Cholec80, JIGSAWS for surgical).

## How the prior feeds the sim
The `TrajectoryPrior` gives the **cutting controller** (`src/roba_sim/control/cutting_controller.py`)
its cut paths, approach angles, and a slice-push starting point; the DiSECt-calibrated force model
supplies the magnitudes. Keep the two streams separate (ADR-005).

Optional deps: `pip install opencv-python numpy` (and the model packages when wiring plug points).
