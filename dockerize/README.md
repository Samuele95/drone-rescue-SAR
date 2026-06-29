# Drone-Rescue SAR Simulation — Dockerized

A decentralised multi-drone search-and-rescue simulation (ROS 2 Jazzy + Gazebo
Harmonic) packaged as a hardened Docker Compose stack. You do **not** need ROS,
Gazebo, or any Python deps on your host — only Docker. Everything is driven by a
single helper script, `./drone.sh`.

## What's in the box

One container image (`drone-rescue-sim:dockerize`) runs six roles:

| Role | What it is | How it runs |
|---|---|---|
| `sim` | Gazebo + the full ~61-node ROS graph (the simulation core) | long-running, `./drone.sh run` |
| `dashboard` | PyQt5 live operator dashboard | GUI profile |
| `rviz` | RViz2 3D view | GUI profile |
| `bench` | headless batch sweep runner — the benchmark | one-shot |
| `mission_control` | PyQt5 launcher GUI (spawns its own sim) | one-shot |
| `report` | PDF report generator over recorded runs | one-shot |

Recorded missions persist in the `drone-rescue_drone-runs` Docker volume.

## Quickstart

```bash
./drone.sh deps        # 1. confirm Docker (+ optional NVIDIA) are present
./drone.sh install     # 2. build the image  (~15-25 min first time)
./drone.sh run         # 3. start the headless simulation
./drone.sh run bench   # 4. run a benchmark mission
./drone.sh tutorial    # full guided walk-through
```

## The `./drone.sh` commands

| Command | Does |
|---|---|
| `deps` | Check host dependencies (Docker, Compose, NVIDIA driver + toolkit, X11, disk, image) and print a pass/fail report. |
| `install` | Build the container image (runs `deps` first). |
| `run [target]` | Operate the stack. Targets: `sim` (default), `gui`, `bench`, `mc`, `report`, `stop`, `status`, `logs`. |
| `delete` | Tear everything down — containers, the runs volume, and the image (asks for confirmation). |
| `readme` | Show this file. |
| `tutorial` | Show the step-by-step tutorial. |
| `help` | Usage summary. |

## Rendering & performance

- With a working **NVIDIA** driver + `nvidia-container-toolkit`, `sim` and the
  GUIs render on the GPU. Without them, the stack falls back to CPU software
  rendering automatically (slower, still correct).
- The **benchmark** deliberately uses software rendering for host-independent,
  reproducible numbers. Validated real-time factor matches/exceeds bare metal
  (RTF 0.3823 vs 0.3665 baseline).

## Going deeper

- `TUTORIAL.md` — guided, copy-paste walk-through from build to results.
- `SECURITY.md` — hardening posture, per-service controls, host changes made.

## Requirements

Docker Engine 25+, Docker Compose v2. Optional for GPU rendering: NVIDIA driver
(matching the running kernel) + `nvidia-container-toolkit` (`./drone.sh deps`
tells you what's missing). GUI windows additionally need an X server / XWayland
on the host.
