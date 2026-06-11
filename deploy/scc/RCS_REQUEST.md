# Draft request to BU Research Computing Services (SCC)

Send to: help@scc.bu.edu (or the RCS help portal). Adjust as you like.

---

**Subject:** Isaac Sim RTX renderer fails on GPU driver 595.71.05 — is a ~580-driver GPU node available?

Hi RCS team,

I'm running NVIDIA Isaac Sim 6.0 (via Singularity, project `pi-brout`) on the SCC GPU nodes for a
robotics-simulation project. Headless physics works fine, but Isaac Sim's **RTX/Vulkan renderer fails
to initialize** on the current driver — `vkCreateDevice` returns `ERROR_INITIALIZATION_FAILED` and no
render device can be created. I've confirmed the GPU-node driver is **595.71.05** (e.g. on the L40S
nodes scc-502/504/510).

This matches a known upstream regression: NVIDIA Isaac Sim 5.1/6.0 fail to initialize the renderer on
the **595 driver series**, and the documented fix is to use the **580** driver line
(github.com/isaac-sim/IsaacSim issue #537). I cannot change the driver myself.

Questions:
1. Do any SCC GPU nodes with RT-core GPUs (A40 / A6000 / L40S / RTX6000ada) currently run a driver in
   the **~580–585** range that I could target (e.g. via a node feature / `-l` resource)?
2. If not, is a driver of that vintage something RCS could make available on a subset of GPU nodes, or
   is there a recommended path for Isaac Sim / Omniverse RTX rendering on the SCC?

For context, I only need the renderer for interactive visualization; the physics simulation itself runs
headless without it. Happy to provide the full crash log.

Thanks!
