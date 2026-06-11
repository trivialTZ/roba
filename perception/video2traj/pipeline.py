"""Video → kinematic trajectory prior (Aim 2.3, stage 1 of ADR-005).

THE SCOPE (verified, docs/FEASIBILITY.md Pillar 10): from ordinary RGB cutting video you can
recover *kinematics* — the knife/hand 2D/3D trajectory, motion phases, slice-vs-press patterns —
but NOT quantitative forces (ill-posed; the blade is occluded inside the tissue). So this pipeline
outputs a **trajectory/strategy prior only**. Forces come from the separate sensor+sim stage
(DiSECt calibration), never from here.

The pipeline is staged with pluggable components. The cheap, model-free parts (frame extraction,
dense optical flow, motion-saliency tool tracking) work out of the box. The learned components
(hand pose, tool segmentation, monocular depth, VLM phase labels) are declared with clear plug
points and recommended models — wire them on SCC where the weights/GPU live.

Optional deps: opencv-python (cv2), numpy. Without them the module still imports; methods raise a
clear message telling you what to install.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Tuple

try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None  # type: ignore
try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore


@dataclass
class TrajectorySample:
    t: float                       # seconds
    tool_xy: Tuple[float, float]   # normalized image coords (0..1) of the blade tip estimate
    flow_mag: float                # mean optical-flow magnitude (motion intensity proxy)
    phase: str = "unknown"         # approach / press / slice / retract (filled by phase labeler)


@dataclass
class TrajectoryPrior:
    """The output artifact: a kinematic/strategy prior, NOT a force model."""

    source: str
    fps: float
    samples: List[TrajectorySample] = field(default_factory=list)
    notes: str = "kinematics only; forces require sensor+sim (ADR-005)"

    def to_json(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump({**asdict(self), "samples": [asdict(s) for s in self.samples]}, f, indent=2)

    @property
    def slice_push_estimate(self) -> Optional[float]:
        """Crude press-vs-slice ratio from horizontal/vertical motion split during cutting phases.

        A real implementation conditions on labeled 'press'/'slice' phases; here we return the
        fraction of total motion that is horizontal as a first-order strategy descriptor.
        """
        if np is None or not self.samples:
            return None
        xs = np.array([s.tool_xy[0] for s in self.samples])
        ys = np.array([s.tool_xy[1] for s in self.samples])
        dh = float(np.sum(np.abs(np.diff(xs))))
        dv = float(np.sum(np.abs(np.diff(ys))))
        return dh / (dh + dv + 1e-9)


class VideoTrajectoryPipeline:
    def __init__(self, sample_fps: float = 10.0):
        self.sample_fps = sample_fps

    # ---- stage 1: frames (model-free) -------------------------------------------------
    def extract_frames(self, video_path: str):
        _need(cv2, "opencv-python")
        cap = cv2.VideoCapture(video_path)
        src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        stride = max(1, int(round(src_fps / self.sample_fps)))
        frames, i = [], 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i % stride == 0:
                frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            i += 1
        cap.release()
        return frames

    # ---- stage 2: motion + tool track (model-free baseline) ---------------------------
    def track(self, video_path: str) -> TrajectoryPrior:
        """Baseline trajectory: dense optical flow + motion-saliency centroid as the tool proxy.

        This is a *baseline* good enough to validate the pipeline plumbing. Replace
        ``_tool_point`` with a real blade/hand detector (see plug points) for research quality.
        """
        _need(cv2, "opencv-python"); _need(np, "numpy")
        frames = self.extract_frames(video_path)
        prior = TrajectoryPrior(source=video_path, fps=self.sample_fps)
        prev_gray = None
        for k, frame in enumerate(frames):
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            flow_mag = 0.0
            xy = (0.5, 0.5)
            if prev_gray is not None:
                flow = cv2.calcOpticalFlowFarneback(
                    prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                mag = np.linalg.norm(flow, axis=2)
                flow_mag = float(mag.mean())
                xy = self._tool_point(mag, gray.shape)
            prior.samples.append(TrajectorySample(t=k / self.sample_fps, tool_xy=xy, flow_mag=flow_mag))
            prev_gray = gray
        self._label_phases(prior)
        return prior

    @staticmethod
    def _tool_point(motion_mag, shape) -> Tuple[float, float]:
        """Centroid of the high-motion region → blade-tip proxy (normalized coords)."""
        h, w = shape
        thr = motion_mag.mean() + motion_mag.std()
        ys, xs = np.where(motion_mag >= thr)
        if len(xs) == 0:
            return 0.5, 0.5
        return float(xs.mean()) / w, float(ys.mean()) / h

    def _label_phases(self, prior: TrajectoryPrior) -> None:
        """Heuristic phase labels from motion magnitude. Replace with a VLM/temporal model."""
        if not prior.samples:
            return
        mags = [s.flow_mag for s in prior.samples]
        hi = (max(mags) + sum(mags) / len(mags)) / 2
        for s in prior.samples:
            s.phase = "slice" if s.flow_mag >= hi else ("press" if s.flow_mag > 0 else "approach")

    # ---- plug points (wire real models on SCC) ----------------------------------------
    def hand_pose(self, frames):
        """PLUG: 3D hand pose per frame. Recommended: MediaPipe Hands (2D) or a 3D hand model
        (HaMeR / MANO-based). See ADR-005, docs/REFERENCES.md Pillar 10."""
        raise NotImplementedError("wire a hand-pose model (MediaPipe / HaMeR) on SCC")

    def tool_segmentation(self, frames):
        """PLUG: per-frame knife/instrument mask → precise blade-tip. Recommended: SAM2 prompt
        on the blade, or a surgical-instrument segmenter for surgical footage."""
        raise NotImplementedError("wire a tool segmenter (SAM2 / instrument seg) on SCC")

    def monocular_depth(self, frames):
        """PLUG: metric/relative depth to lift 2D tracks to 3D. Recommended: Depth-Anything-V2.
        Needed to resolve the scale ambiguity in monocular trajectories (Pillar 10)."""
        raise NotImplementedError("wire a monocular depth model (Depth-Anything) on SCC")


def _need(mod, pipname: str) -> None:
    if mod is None:
        raise RuntimeError(f"{pipname} not installed — `pip install {pipname}` in the perception env")
