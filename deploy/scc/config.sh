#!/usr/bin/env bash
# Shared config for roba SCC scripts.  `source config.sh`  before the others.
# Verified against the live cluster (project pi-brout, user tztang) on 2026-06-10.
# Everything big lives on /projectnb (home is 10 GB hard cap — already ~6.4 GB used). See SCC_USAGE.md.

# ---- cluster facts (edit only if your account differs) ----
export ROBA_PROJECT="${ROBA_PROJECT:-pi-brout}"          # SCC project (-P and /projectnb/<proj>)
export ROBA_GPU_TYPE="${ROBA_GPU_TYPE:-A40}"             # A40/A6000/L40S/RTX6000ada (RT cores). NEVER A100/H200/V100/P100.
export ISAAC_VER="${ISAAC_VER:-6.0.0}"                    # pin; drop to 5.1.0 if the GPU-node driver < 580.65.06

# ---- derived paths (all on /projectnb scratch) ----
export ROBA_SCRATCH="/projectnb/${ROBA_PROJECT}/${USER}/roba_work"
export ROBA_SIF="${ROBA_SCRATCH}/sif/isaac-sim-${ISAAC_VER}.sif"
export ROBA_CACHE="${ROBA_SCRATCH}/cache/isaac"
export ROBA_REPO="/projectnb/${ROBA_PROJECT}/${USER}/roba"   # the cloned repo

# Singularity build/runtime caches MUST be off /tmp and off home (a 20 GB pull will overflow them).
export SINGULARITY_CACHEDIR="${ROBA_SCRATCH}/cache/singularity"
export SINGULARITY_TMPDIR="${ROBA_SCRATCH}/cache/singularity-tmp"

# Guard: refuse heavy steps on a login node (15-min CPU limit -> killed/flagged). Inside an SGE
# batch/interactive job, $JOB_ID is set.
roba_require_compute_node() {
  if [ -z "${JOB_ID:-}" ]; then
    echo "REFUSING: not inside a batch/interactive job (no \$JOB_ID)." >&2
    echo "  Login nodes kill >15 min CPU and risk a policy flag. Start a job first, e.g.:" >&2
    echo "    qrsh -P ${ROBA_PROJECT} -l gpus=1 -l gpu_type=${ROBA_GPU_TYPE} -l h_rt=4:00:00 -pe omp 4" >&2
    return 1
  fi
}

mkdir -p "${ROBA_SCRATCH}/sif" "${ROBA_CACHE}" "${SINGULARITY_CACHEDIR}" "${SINGULARITY_TMPDIR}" 2>/dev/null
echo "roba SCC config: project=${ROBA_PROJECT} gpu=${ROBA_GPU_TYPE} isaac=${ISAAC_VER}"
echo "  sif=${ROBA_SIF}"
echo "  scratch=${ROBA_SCRATCH}  (home is 10 GB-capped — keep big files here)"
