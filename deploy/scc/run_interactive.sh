#!/usr/bin/env bash
# Launch Isaac Sim interactively (WebRTC livestream) for the mouse-driven demo.
# MUST run INSIDE a GPU job. Release it the moment you stop working (idle GPUs are antisocial).
#
#   qrsh -P pi-brout -l gpus=1 -l gpu_type=A40 -l h_rt=4:00:00 -pe omp 4
#   source config.sh && bash run_interactive.sh
#   exit            # as soon as you're done
#
# Connect from the Mac via SCC OnDemand desktop or an SSH-forwarded WebRTC port (validate in Phase 0.1).
set -euo pipefail
cd "$(dirname "$0")" && source ./config.sh
roba_require_compute_node || exit 1
[ -f "$ROBA_SIF" ] || { echo "No image at $ROBA_SIF — run 00_build_sif.sh first."; exit 1; }

# Bind every Isaac Sim cache to /projectnb so nothing writes to the 10 GB home.
BINDS=(
  -B "${ROBA_CACHE}/kit:/isaac-sim/.cache"
  -B "${ROBA_CACHE}/computecache:/isaac-sim/.nv/ComputeCache"
  -B "${ROBA_CACHE}/logs:/isaac-sim/.nvidia-omniverse/logs"
  -B "${ROBA_CACHE}/ovconfig:/isaac-sim/.nvidia-omniverse/config"
  -B "${ROBA_CACHE}/ovdata:/isaac-sim/.local/share/ov/data"
  -B "${ROBA_CACHE}/ovpkg:/isaac-sim/.local/share/ov/pkg"
  -B "${ROBA_CACHE}/hub:/var/cache/hub"
  -B "${ROBA_REPO}:/workspace/roba"
  -B "${ROBA_SCRATCH}/assets:/workspace/assets"
)
mkdir -p "${ROBA_CACHE}"/{kit,computecache,logs,ovconfig,ovdata,ovpkg,hub}

echo "Launching Isaac Sim ${ISAAC_VER} (WebRTC livestream). GPU:"; nvidia-smi -L || true
singularity exec --nv \
  --env ACCEPT_EULA=Y --env PRIVACY_CONSENT=N --env ROBA_ASSET_ROOT=/workspace/assets/robots \
  "${BINDS[@]}" \
  "$ROBA_SIF" \
  /isaac-sim/runheadless.sh -v      # swap for: /isaac-sim/python.sh /workspace/roba/experiments/run_demo.py
