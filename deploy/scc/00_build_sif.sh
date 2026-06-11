#!/usr/bin/env bash
# Build the Isaac Sim Singularity image (.sif) from the NGC Docker image.
# MUST run INSIDE a job (CPU+IO heavy, pulls ~20 GB — login nodes kill it at 15 min CPU).
#
#   1) source config.sh
#   2) start a CPU job (no GPU needed to build):
#        qrsh -P pi-brout -l h_rt=2:00:00 -pe omp 8
#   3) bash 00_build_sif.sh
#
# NGC API key (free): https://ngc.nvidia.com -> Setup -> Generate API Key.
set -euo pipefail
cd "$(dirname "$0")" && source ./config.sh
roba_require_compute_node || exit 1

if [ -f "$ROBA_SIF" ]; then
  echo "Image already exists: $ROBA_SIF  (delete to rebuild)"; exit 0
fi

if [ -z "${SINGULARITY_DOCKER_PASSWORD:-}" ]; then
  export SINGULARITY_DOCKER_USERNAME='$oauthtoken'
  read -rsp "Paste your NGC API key (hidden): " SINGULARITY_DOCKER_PASSWORD; echo
  export SINGULARITY_DOCKER_PASSWORD
fi

echo "Building $ROBA_SIF from docker://nvcr.io/nvidia/isaac-sim:${ISAAC_VER}"
echo "(cache/tmp on /projectnb so the 20 GB pull never touches the 10 GB home)"
singularity build "$ROBA_SIF" "docker://nvcr.io/nvidia/isaac-sim:${ISAAC_VER}"
echo "Done: $ROBA_SIF ($(du -h "$ROBA_SIF" | cut -f1))"
