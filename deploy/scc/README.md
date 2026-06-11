# Running roba's Isaac Sim on BU SCC

Isaac Sim **cannot run on macOS** — the Mac is a thin client. Compute happens on **SCC A40/A6000**
nodes (RT-core GPUs → Isaac Sim-supported; A100/H200 are **not** supported). **Read
[`SCC_USAGE.md`](SCC_USAGE.md) first** — it's the don't-get-banned guide, and the scripts enforce its
key rules (no login-node compute; everything big on `/projectnb`, not the 10 GB home).

## One-time setup
1. **Edit [`config.sh`](config.sh)** — set `ROBA_PROJECT` (your SCC project) and confirm `ROBA_GPU_TYPE=A40`.
2. **Check the driver.** On an A40 node: `nvidia-smi | head`. If the driver is **< 580.65.06**, set
   `ISAAC_VER=5.1.0` in `config.sh` (Isaac Sim 6.0 needs the newer driver).
3. **Build the image inside a job** (never on login):
   ```bash
   qrsh -P <your_project> -l h_rt=2:00:00 -pe omp 8     # CPU job is enough to build
   source config.sh && bash 00_build_sif.sh             # prompts for your NGC API key
   exit                                                  # release the job
   ```

## Interactive demo (mouse-driven cutting)
```bash
qrsh -P <your_project> -l gpus=1 -l gpu_type=A40 -l h_rt=4:00:00 -pe omp 4
source config.sh && bash run_interactive.sh
# connect from your Mac via SCC OnDemand desktop or a forwarded WebRTC port (validate in Phase 0.1)
exit            # <-- the instant you stop working, so you never park an idle GPU
```
For the smoothest GUI, launch this **inside an SCC OnDemand "Desktop / Virtual GPU Desktop"** session.

## Long headless runs (training, DiSECt calibration, batch rendering)
```bash
qsub job_headless.qsub experiments/your_script.py     # GPU released automatically when it finishes
qstat -u $USER                                        # watch it; qdel <id> to cancel
```

## Files
| File | Purpose |
|------|---------|
| `SCC_USAGE.md` | **don't-get-banned guide** (login nodes, quotas, idle GPUs, fair-share) |
| `config.sh` | shared config; forces `/projectnb` paths; login-node guard |
| `00_build_sif.sh` | build the Isaac Sim `.sif` (inside a job, cache on scratch) |
| `run_interactive.sh` | WebRTC-livestream interactive session |
| `job_headless.qsub` | SGE batch template for headless runs |

> These are **Phase 0.1 starting templates** to validate on SCC, not yet battle-tested there.
> Expect to adjust the Apptainer bind set, the module names (`apptainer` vs `singularity`), and the
> connectivity path once you run them. Record what works in `experiments/ENV.md`.
