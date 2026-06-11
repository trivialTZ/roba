"""GPU-free USD introspection — find articulation root, joints, and link names.

Pure pxr.Usd (no SimulationApp, no renderer, no GPU) → sidesteps the Vulkan/renderer issue and
runs on a CPU node. Use it to learn the real link names so we can set config.OPENARM_CUTTING.ee_frame.

Usage:  python usd_inspect.py <path-to.usd>
"""
import sys

from pxr import Usd, UsdPhysics  # provided by the Isaac Sim container

path = sys.argv[1] if len(sys.argv) > 1 else ""
print(f"opening: {path}", flush=True)
stage = Usd.Stage.Open(path)
if stage is None:
    print("FAILED to open stage"); sys.exit(1)

art_roots, joints, rigid_bodies, xforms = [], [], [], []
for prim in stage.Traverse():
    p = prim.GetPath().pathString
    t = prim.GetTypeName()
    schemas = list(prim.GetAppliedSchemas())
    if any("ArticulationRoot" in s for s in schemas):
        art_roots.append(p)
    if "Joint" in str(t):
        jt = "fixed" if "Fixed" in str(t) else ("revolute" if "Revolute" in str(t) else str(t))
        joints.append((p, jt))
    if any("RigidBodyAPI" in s for s in schemas):
        rigid_bodies.append(p)
    if t == "Xform":
        xforms.append(p)

print("\n=== ARTICULATION ROOT(S) ===", flush=True)
for p in art_roots:
    print("  ", p, flush=True)
print(f"\n=== RIGID BODIES ({len(rigid_bodies)}) — these are the links ===", flush=True)
for p in rigid_bodies:
    print("  ", p, flush=True)
print(f"\n=== JOINTS ({len(joints)}) ===", flush=True)
for p, t in joints:
    print("  ", t, p, flush=True)
print(f"\n=== leaf Xforms (candidate EE frames, last 25) ===", flush=True)
for p in xforms[-25:]:
    print("  ", p, flush=True)
print("\nUSD_INSPECT OK", flush=True)
