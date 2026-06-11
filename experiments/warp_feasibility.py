"""Decisive GPU-compute feasibility probe for the deformable-cutting path (DiSECt/MPM/Warp).

Runs in the Isaac Sim container on a 595-driver GPU node. Confirms (a) NVIDIA Warp runs a CUDA
kernel (Warp is bundled in the container → a Warp-native deformable cut needs NO new install),
and (b) torch-CUDA + nvcc availability (informs whether reviving DiSECt's dflex backend is viable
in-container). No SimulationApp / renderer needed — pure compute, sidesteps the Vulkan/RTX block.
Output -> $ROBA_OUT.
"""
import json, os, shutil, traceback

res = {"warp_ok": False}

try:
    import numpy as np
    import warp as wp
    res["warp_version"] = getattr(wp, "__version__", getattr(wp.config, "version", "?"))
    wp.init()
    res["cuda_device_count"] = int(wp.get_cuda_device_count())

    @wp.kernel
    def k(a: wp.array(dtype=wp.float32), b: wp.array(dtype=wp.float32)):
        i = wp.tid()
        b[i] = a[i] * 2.0 + 1.0

    n = 10000
    a = wp.array(np.arange(n, dtype=np.float32), device="cuda")
    b = wp.zeros(n, dtype=wp.float32, device="cuda")
    wp.launch(k, dim=n, inputs=[a, b], device="cuda")
    wp.synchronize()
    val = float(b.numpy()[5])           # expect 5*2+1 = 11
    res["kernel_value_at_5"] = val
    res["warp_ok"] = abs(val - 11.0) < 1e-3
    # does warp ship sim/FEM/MPM modules we could build a deformable cut on?
    res["has_warp_sim"] = bool(__import__("importlib").util.find_spec("warp.sim"))
    res["has_warp_fem"] = bool(__import__("importlib").util.find_spec("warp.fem"))
except Exception:
    res["error_warp"] = traceback.format_exc()

# torch + nvcc probe (for the DiSECt/dflex route)
try:
    import torch
    res["torch"] = torch.__version__
    res["torch_cuda"] = bool(torch.cuda.is_available())
    res["torch_cuda_dev"] = torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
except Exception as e:
    res["torch"] = f"n/a: {e}"
res["nvcc_in_container"] = shutil.which("nvcc")

out = os.environ.get("ROBA_OUT", "/tmp")
os.makedirs(out, exist_ok=True)
json.dump(res, open(os.path.join(out, "warp_feasibility.json"), "w"), indent=2)
print("WARP_OK" if res.get("warp_ok") else "WARP_FAIL", flush=True)
print(json.dumps(res, indent=2), flush=True)
