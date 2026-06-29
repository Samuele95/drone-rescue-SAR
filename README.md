# Drone-Rescue: Stigmergic Multi-Drone Search & Rescue

Drone-Rescue is a high-fidelity simulation of a fleet of autonomous drones that
searches an earthquake-stricken area to locate survivors. A swarm of drones
spreads out over the disaster zone, coordinates without any central controller,
fuses noisy sightings into confirmed victim locations, and keeps working as
individual drones run low on battery or drop out. It runs on ROS 2 Jazzy and the
Gazebo Harmonic physics engine, and ships as a one-command Docker stack.

It is built for two kinds of user: an **operator** who configures and watches a
single rescue mission through a desktop interface, and an **analyst** who runs
batches of missions headlessly and compares search strategies on hard numbers.

---

## Highlights

- **Decentralised coordination.** Drones steer themselves from a shared
  "pheromone" field rather than taking orders from a hub, so the fleet has no
  single point of failure and keeps searching when drones are lost.
- **Automatic victim handling.** Uncertain sightings are clustered and fused into
  confirmed detections; each confirmed victim is auctioned to the nearest capable
  drone and investigated as a self-contained, recoverable task.
- **Operator dashboard.** A live view of coverage, per-drone battery and health,
  a scan-time estimate, and a flight-plan go/no-go indicator, with a 3D scene in
  RViz.
- **Reproducible experiments.** Any mission replays deterministically from a seed;
  a headless runner sweeps search patterns across scenarios and produces
  comparison plots and a PDF report.
- **Safety-aware.** Drones honour no-fly zones and altitude limits, and a drone
  that can no longer reach its target and return home is flagged before it fails.
- **Runs anywhere Docker does.** No ROS or Gazebo install on the host — one helper
  script builds, runs, and tears the whole thing down.

## Getting started

You only need Docker (Engine 25+ and Compose v2). An NVIDIA GPU is optional and
speeds up rendering; without one the simulation falls back to software rendering
automatically.

```bash
./drone.sh deps        # check your machine has what it needs
./drone.sh install     # build the container image (first build ~15-25 min)
./drone.sh run         # start a headless simulation (ready in ~15s)
./drone.sh run gui     # open the operator dashboard and 3D view (needs a display)
./drone.sh tutorial    # a full, copy-paste walk-through
```

Run a batch of missions and generate a report:

```bash
./drone.sh run bench --patterns spiral_out,parallel_track --scenarios default,cluster --trials 3 --runs-dir /data/runs/my_sweep
./drone.sh run report --sweep /data/runs/my_sweep --out /data/runs/my_sweep/report.pdf
```

When you're done, `./drone.sh delete` removes the containers, recorded runs, and
image (it asks for confirmation first).

## How a mission works

A mission begins with the fleet spreading out under a chosen search pattern —
spiral, parallel sweep, expanding squares, and others. As drones fly, each
deposits and reads a shared field that biases the others away from already-covered
ground, so coverage grows without a central plan. When a drone sees something that
might be a victim, the sighting joins a cluster; once enough independent sightings
agree, the victim is confirmed and the nearest available drone is dispatched to
investigate and confirm it up close. Throughout, the system tracks coverage,
fairness of work across the fleet, battery use, and how long each victim waited to
be found — the numbers an analyst later compares between strategies.

## Configuration

Missions are configured entirely through declarative YAML — no code changes
needed. A scenario file sets the fleet size, the search pattern, victim
placements, and environmental conditions; a library of ready-made scenarios
(`default`, `cluster`, `sparse`, `hazard`, `low_visibility`, `dying_swarm`,
`sloped_terrain`) each varies one dimension off the baseline. Base parameter files
cover the drones, the pheromone field, the weather/wind model, and the no-fly
zones. See `dockerize/RUNBOOK.md` for the full parameter reference.

## What's inside

The system is a set of ROS 2 packages under `drone_rescue_ws/src/`:

| Package | Responsibility |
|---|---|
| `drone_rescue_coordination` | The coordination logic: per-drone control plus the search, auction, detection-fusion, rescue-task, feasibility, wind, terrain, and no-fly-zone behaviour. |
| `drone_rescue_bringup` | Launch files and all declarative configuration, including the scenario library. |
| `drone_rescue_gazebo` | The Gazebo worlds and drone/world 3D models. |
| `drone_rescue_dashboard` | The live operator dashboard. |
| `drone_rescue_mission_control` | The mission launcher, recorder, batch-sweep runner, and PDF report generator. |
| `drone_rescue_viz` | RViz overlays — pheromone heatmap, drone trails, victim markers, telemetry. |
| `drone_rescue_msgs` | The message contracts the components communicate over. |
| `drone_rescue_ui_common` | Shared interface styling and view models. |
| `ros_tcp_endpoint` | An optional Unity bridge. |

Under the hood the design keeps the decision-making logic in a small, pure-Python
core that is independent of ROS, with the simulator and middleware attached at the
edges — which is what lets the same logic be tested in isolation and, in
principle, moved onto real hardware by swapping the adapters.

## Tests

The core logic is plain Python and runs without a simulator:

```bash
cd drone_rescue_ws
PYTHONPATH=src/drone_rescue_coordination:$PYTHONPATH python3 -m pytest src/*/test
```

## Documentation

| Document | What it covers |
|---|---|
| `dockerize/TUTORIAL.md` | Step-by-step walk-through from build to results. |
| `dockerize/RUNBOOK.md` | Operating the stack: prerequisites, parameters, backups, troubleshooting, upgrades. |
| `docs/studyguide/` | A concise illustrated guide to the internal architecture and the algorithms behind the coordination (build to `main.pdf`). |
| `docs/thesis/` | The full, in-depth technical report, including the evaluation study. |
| `dockerize/REPORT.md`, `dockerize/SECURITY.md` | Container build validation and security posture. |
