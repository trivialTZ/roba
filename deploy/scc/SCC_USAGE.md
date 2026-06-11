# BU SCC usage — how to run Isaac Sim without getting flagged

The user's explicit constraint: **don't get banned.** On a 3,000-user fair-share cluster, flags come
from *how* you run, not how much. These are the rules, grounded in BU RCS policy (sources at bottom).
All scripts in this folder enforce the load-bearing ones automatically.

## The five rules that actually get people warned

1. **NEVER run heavy work on a login node.** Login nodes (`scc1`, `scc2`, `scc4`, …) **kill any process
   over 15 minutes of CPU time** and are for editing/transfer/compile/light-debug only. This includes the
   **Apptainer image build** (CPU+IO heavy, pulls ~20 GB) and **Isaac Sim itself**. → Always `qsub`/`qrsh`
   to a compute node first. *Every script here refuses to run unless `$JOB_ID` is set (i.e. inside a job).*

2. **Keep big files out of `$HOME` (hard 10 GB cap, cannot be raised).** The Isaac Sim `.sif` (~10–20 GB)
   + caches will instantly blow the home quota, which then **breaks all your jobs**. → Put the `.sif`,
   Apptainer cache/tmp, and *all* Isaac Sim runtime caches under **`/projectnb/<project>/$USER/`**.
   *Scripts set `APPTAINER_CACHEDIR`/`APPTAINER_TMPDIR` and bind every cache to `/projectnb`.*

3. **Request the minimum, set a real walltime.** One GPU (`-l gpus=1`), a few cores (`-pe omp 4`), and an
   honest `-l h_rt`. Don't grab multiple A40s or a 24 h block "just in case." Over-reservation on scarce
   GPUs is exactly what fair-share penalizes.

4. **Never park an idle GPU.** Interactive GPU sessions hold a scarce resource the whole time. When you
   stop actively working, **`exit` (or `qdel <job_id>`) immediately** — don't leave the sim open overnight.
   Use **batch (`qsub`)** for long headless runs so the GPU is released the instant the job finishes.

5. **Use your project and stay in your lane.** Always `-P <project>` for correct accounting. Don't flood
   the queue with hundreds of array jobs; space out parameter sweeps. (GPUs are currently "friendly-user"
   mode — only CPU is charged — but fair-share and etiquette still apply.)

## Quick reference

| Thing | Wrong (ban-risk) | Right |
|------|------------------|-------|
| Build `.sif` | on login node | inside `qrsh`/`qsub` job, cache on `/projectnb` |
| Store image | `~/` (10 GB cap) | `/projectnb/<project>/$USER/sif/` |
| Run sim | `./runheadless.sh` on login | inside a GPU job |
| Interactive GPU | leave open idle | `exit` the moment you stop working |
| Long training | interactive overnight | `qsub` batch with `-l h_rt` |
| GPU request | `gpus=4`, A100/H200 | `gpus=1 gpu_type=A40` |

## Disk layout (set once)

```
/projectnb/<project>/$USER/
  sif/        isaac-sim-6.0.0.sif        # the container image
  cache/      apptainer build cache + Isaac Sim runtime caches
  roba/       a clone of this repo (code/configs)
~/            small: dotfiles, scripts, git. NEVER the .sif or caches.
```

## Monitoring your own usage (run these, stay aware)

- `qstat -u $USER` — your running/queued jobs (kill stragglers with `qdel <id>`).
- `pquota -u <project>` / `quota -s` — project & home disk usage (catch home before it fills).
- `acctool -b <date> <project>` — your accounting/usage history.
- `module load gpu-util` then `nvidia-smi` *inside the job* — confirm you're actually using the GPU you reserved (idle reserved GPUs are the antisocial case).

## Sources
- [SCC Best Practices](https://www.bu.edu/tech/support/research/system-usage/running-jobs/best-practices/)
- [Access & Security Policies](https://www.bu.edu/tech/support/research/system-usage/connect-scc/access-and-security/) · [BU Acceptable Use Policy](https://www.bu.edu/policies/acceptable-use-of-computing-services-policy/)
- [Storage Quotas](https://www.bu.edu/tech/support/research/system-usage/using-file-system/storage-quotas/) · [Project Disk Space](https://www.bu.edu/tech/support/research/computing-resources/file-storage/proj-diskspace/)
- [Running Jobs / GPU options](https://www.bu.edu/tech/support/research/system-usage/running-jobs/resources-jobs/) · [Using Singularity/Apptainer](https://www.bu.edu/tech/support/research/software-and-programming/containers/singularity/)
