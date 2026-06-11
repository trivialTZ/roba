"""Per-frame scene recording for driver-agnostic playback.

Workaround for the SCC 595-driver RTX block (docs FEASIBILITY / ENV): Isaac's renderer won't start,
but headless physics runs fine. So we record the world pose of each box prim every frame and replay
the geometry later with a plain matplotlib/USD renderer that needs no NVIDIA driver.

Every recorded object is a box (the slab sub-blocks, the blade, optionally arm-link proxies). We
store, per frame, each box's world translation (3) + the upper-left 3x3 of its local-to-world matrix
(9, rotation*scale, USD row-vector convention). The renderer maps the unit-cube corners (±0.5, since
our prims are UsdGeom.Cube size=1) by:  world = local @ M3 + t.

JSON output is small (~tens of boxes × 12 floats × a few dozen frames) and trivially portable.
"""
from __future__ import annotations

import json
import os
from typing import List, Optional, Tuple

try:
    from pxr import UsdGeom
except Exception:  # pragma: no cover
    UsdGeom = None  # type: ignore


class Recorder:
    def __init__(self, stage):
        if UsdGeom is None:
            raise RuntimeError("pxr not available — Recorder runs inside Isaac Sim")
        self.stage = stage
        self._paths: List[str] = []
        self.meta: List[dict] = []      # per box: {name, color}
        self.frames: List[List[List[float]]] = []
        self._xcache = UsdGeom.XformCache()

    def add_box(self, path: str, color: Tuple[float, float, float], name: Optional[str] = None) -> None:
        """Register a UsdGeom.Cube (size=1) prim to record. Scale/rotation is read from its transform."""
        self._paths.append(path)
        self.meta.append({"name": name or path.split("/")[-1], "color": [float(c) for c in color]})

    def capture(self) -> None:
        self._xcache.Clear()
        frame = []
        for p in self._paths:
            prim = self.stage.GetPrimAtPath(p)
            m = self._xcache.GetLocalToWorldTransform(prim)  # Gf.Matrix4d, row-major, row-vector conv
            t = [float(m[3][0]), float(m[3][1]), float(m[3][2])]
            m3 = [float(m[i][j]) for i in range(3) for j in range(3)]
            frame.append(t + m3)  # 12 floats
        self.frames.append(frame)

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump({"meta": self.meta, "frames": self.frames}, f)
