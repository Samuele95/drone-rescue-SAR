# Tutorial — running the Drone-Rescue simulation through Docker

A copy-paste walk-through from a clean machine to benchmark results and a PDF
report. Everything goes through `./drone.sh` (which wraps Docker Compose). Run
it from the project root.

---

## Step 0 — Check your machine

```bash
./drone.sh deps
```

You need **Docker** and **Docker Compose v2** (mandatory — the command stops if
they're missing). NVIDIA driver + `nvidia-container-toolkit` are *optional*:
present → GPU rendering; absent → automatic CPU software-rendering fallback.
An X server is only needed later for the GUI windows.

What you'll see: a checklist with `✓` (good), `!` (optional/degraded), or `✗`
(blocking). Fix any `✗` before continuing.

---

## Step 1 — Build the image

```bash
./drone.sh install
```

This builds `drone-rescue-sim:dockerize` — ROS 2 Jazzy, Gazebo Harmonic, the
colcon workspace, and a baked Gazebo Fuel-model cache. **First build takes
15-25 minutes** (it compiles the workspace and pre-fetches world models);
subsequent builds are cached and fast. You only do this once.

---

## Step 2 — Start the simulation (headless)

```bash
./drone.sh run
```

Brings up the `sim` service and waits until its healthcheck passes (Gazebo +
the `pheromone_server` node alive) — usually ~15 seconds. Then it prints the
container status.

Watch it work:

```bash
./drone.sh run logs       # follow the live ROS log (Ctrl-C to stop watching)
./drone.sh run status     # show container health
```

This is the simulation running. It is headless (no window) — drones spawn,
coordinate via the shared pheromone field, and search the modelled earthquake
zone. To *see* it, use the GUI in Step 4.

Stop it when done:

```bash
./drone.sh run stop
```

---

## Step 3 — Run a benchmark mission

The benchmark runs a full mission end-to-end and records a JSON result with a
measured performance envelope (real-time factor, CPU, memory). It spawns its own
simulation, so `./drone.sh run bench` stops the long-running `sim` first.

Defaults (one mission, spiral search, default scenario):

```bash
./drone.sh run bench
```

Or pass your own sweep (patterns × scenarios × trials):

```bash
./drone.sh run bench \
  --patterns spiral_out,parallel_track,random_walk \
  --scenarios default,cluster --trials 3 \
  --runs-dir /data/runs/my_sweep
```

A mission takes ~25-30 min of wall time at software-render speed. When it
finishes you'll see a one-line summary like
`OK (1601s) ... cov=35.8% TP=1 FP=5 F1=0.18`.

---

## Step 4 — See it live (GUI)

Needs an X server / XWayland on the host. The command grants local X access for
you (`xhost +local:`), then starts the sim plus the dashboard and RViz windows:

```bash
./drone.sh run gui
```

Two windows open: the **dashboard** (live coverage, fleet health, the scan-time
ETA and flight-plan go/no-go indicators) and **RViz** (3D view with the
pheromone heatmap, drone trails, and victim markers).

For the interactive launcher GUI (choose scenario, launch, watch) instead:

```bash
./drone.sh run mc
```

Tidy up the X grant afterwards with `xhost -local:`.

---

## Step 5 — Get your results out

Recorded runs live in the `drone-rescue_drone-runs` Docker volume. Generate a
PDF report over a sweep directory:

```bash
./drone.sh run report --sweep /data/runs/my_sweep --out /data/runs/my_sweep/report.pdf
```

Copy the whole runs volume to your host (`./out/`):

```bash
docker run --rm -v drone-rescue_drone-runs:/d -v "$PWD/out":/o alpine \
  sh -c 'cp -r /d/* /o/ && chown -R '"$(id -u):$(id -g)"' /o'
```

---

## Step 6 — Clean up

Stop the stack but keep your recorded runs and the image:

```bash
./drone.sh run stop
```

Remove **everything** — containers, the runs volume (deletes recorded missions!),
and the image (asks you to type `delete` to confirm):

```bash
./drone.sh delete
```

Your source tree is never touched; `./drone.sh install` rebuilds from scratch.

---

## Troubleshooting

| Symptom | Try |
|---|---|
| `sim` never healthy | `./drone.sh run logs`; rebuild (`./drone.sh install`) if Fuel models are missing |
| GUI window doesn't appear | run `xhost +local:`, check `echo $DISPLAY`; on Wayland it goes via XWayland |
| GPU not used / slow | `./drone.sh deps`; after a kernel upgrade reinstall `linux-modules-nvidia-580-open-$(uname -r)` (see RUNBOOK) |
| "Image not built" | `./drone.sh install` |
| Out of disk during build | `docker builder prune`; ensure ~10 GB free |

More detail in `RUNBOOK.md`.
