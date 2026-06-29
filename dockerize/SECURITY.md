# Security posture — drone_rescue containerized stack

Last review: 2026-06-19. Next review due: 2026-09-19.
Target host: native Linux (kernel 6.17.0-35), Docker 29.4.3, NVIDIA RTX 4070 via
nvidia-container-toolkit 1.19.1.

## Threat model

### What this stack defends against
- A compromised simulation/GUI service modifying the rootfs, persisting a backdoor,
  or escalating to root inside the container (`read_only` + `cap_drop: ALL` +
  `no-new-privileges`, non-root UID 10001).
- Resource exhaustion of one service starving others (`cpus`/`mem_limit` per service).
- Network reachability: the `ros` network is `internal: true` — no container can
  reach the public internet at runtime, and nothing is published to the host.
- Build-time supply-chain drift (every base image + named package pinned).

### What this stack does NOT defend against
- Compromise of the Docker daemon or host kernel.
- Supply-chain compromise of upstream base images (mitigated by digest pinning, not eliminated).
- Side-channel attacks across containers on the same host / same GPU.
- A container with the X11 socket mounted (GUI profile) reading the host display
  and input — see the display waiver below.
- Insider with shell access on the host.

## Per-service controls

All services share one image (`drone-rescue-sim:dockerize`, built from
`drone_rescue_ws/Dockerfile.dockerize`) on base
`ros@sha256:6513503d0b10e919fbe8134981d4f9d19b5c1f9b045b87a9fe3b0b9e03e7c2a9`
(ros:jazzy-ros-base, Ubuntu 24.04). Non-root `USER 10001` in all.

| Control | sim | dashboard | rviz | bench | report | mission_control |
|---|---|---|---|---|---|---|
| Tier | compute-hot | worker | worker | compute-hot | worker | worker |
| read_only rootfs | yes | yes | yes | yes | yes | yes |
| User UID | 10001 | 10001 | 10001 | 10001 | 10001 | 10001 |
| cap_drop | ALL | ALL | ALL | ALL | ALL | ALL |
| cap_add | none | none | none | none | none | none |
| no-new-privileges | yes | yes | yes | yes | yes | yes |
| tmpfs | /tmp,/home/app,/tmp/xdg | same | same | same | same | same |
| Seccomp | docker-default | docker-default | docker-default | docker-default | docker-default | docker-default |
| AppArmor | docker-default | docker-default | docker-default | docker-default | docker-default | docker-default |
| Runtime | runc | runc | runc | runc | runc | runc |
| GPU (NVIDIA) | reserved (graphics,compute,utility) | reserved | reserved | none (software) | none | reserved |
| Network | ros (internal) | ros | ros | ros | ros | ros |
| Egress | none | none | none | none | none | none |
| Display (X11) | no | /tmp/.X11-unix ro | /tmp/.X11-unix ro | no | no | /tmp/.X11-unix ro |
| Volumes | drone-runs rw | drone-runs ro | none | drone-runs rw | drone-runs rw | drone-runs rw |
| Healthcheck | gz sim + pheromone_server | pgrep dashboard | pgrep rviz2 | (one-shot) | (one-shot) | (one-shot) |
| Resource limits | 6 CPU / 6G | 1.5 / 1G | 1.5 / 1G | 6 / 6G | 2 / 2G | 6 / 6G |
| shm_size | 2g | 512m | 512m | 2g | default | 2g |
| Logging | json-file 10m×3 | same | same | same | same | same |
| init (tini) | yes | yes | yes | yes | yes | yes |

## Waivers from baseline

| Service | Control | Default | Actual | Reason | Mitigation |
|---|---|---|---|---|---|
| dashboard, rviz, mission_control | host X11 socket mount + `xhost +local:` | no host display access | `/tmp/.X11-unix` mounted ro; local X access granted | GUI profile renders Gazebo/RViz/Qt to the operator's desktop; Wayland-native passthrough is unreliable for OGRE+Qt5 | GUI is opt-in (`--profile gui`); socket mounted read-only; `xhost -local:` revokes after use; default `up` is headless with no display access |
| sim, mission_control | `internal: true` network + GPU reservation | n/a | NVIDIA device reserved | render-only graphics use; no CUDA compute exposed beyond what OGRE needs | `NVIDIA_DRIVER_CAPABILITIES` scoped to `graphics,compute,utility` (not `all`); no `--privileged` |
| dashboard, rviz, mission_control | `ipc: host` | private IPC namespace | shares the host IPC namespace | X11 MIT-SHM: a GUI client's System-V shared-memory segment must be visible to the host X/XWayland server for `XShmAttach` to succeed; with a private IPC namespace it fails (`MESA: Failed to attach to x11 shm`) | GUI profile only (opt-in); does not apply to the headless `sim`/`bench`/`report`; revert by removing `ipc: host` and accepting the (non-fatal) MESA fallback |
| dashboard, rviz, mission_control | `/dev/dri` device + `group_add` render/video | no GPU device in container | render node passed; container user added to host render(992)/video(44) groups | Mesa's `iris` driver needs the DRM render node to create a DRI3 screen for the X window (`failed to load driver: iris`); without it GL fails or drops to slow software | GUI profile only; specific device (not all of `/dev`); group IDs supplied per-host by `drone.sh`; Mesa falls back to software if the device is unusable |

No `read_only` relaxations. No capabilities added. No `--privileged`, no host
networking, no host PID/userns. `ipc: host` is used only by the three GUI
services (above) for X shared-memory; the headless services keep a private IPC.

## Secrets handling
None. The project consumes no secrets.
No `secrets:` blocks; no `.env`; nothing baked into image layers.

## Image supply chain

| Image | Digest pinned? | Scan tool | Critical | High | Top findings |
|---|---|---|---|---|---|
| drone-rescue-sim:dockerize | built locally; base + all apt pkgs pinned | NOT RUN — see below | — | — | — |
| ros:jazzy-ros-base (base) | yes (@sha256:6513503d…) | — | — | — | Ubuntu 24.04 + ROS Jazzy base |

**Image scan not performed:** neither `docker scout` nor `trivy` is installed on
this host, and the skill does not install scanners on the user's behalf.
**Follow-up (recommended before any non-local use):**
```bash
# Option A — Docker Scout (Docker Desktop / engine plugin)
docker scout cves drone-rescue-sim:dockerize
# Option B — Trivy
docker run --rm -v /var/run/docker.sock:/var/run/docker.sock \
  aquasec/trivy:latest image --severity CRITICAL,HIGH drone-rescue-sim:dockerize
```
Record critical/high counts + top findings here, then triage per the policy below.

Base image refresh policy: re-resolve the `ros:jazzy-ros-base` digest and the
pinned apt versions quarterly, and on any critical CVE in the runtime base;
rebuild and re-run the Phase-5 benchmark to confirm parity held.

## Network posture

| Network | Internal | Members | Egress |
|---|---|---|---|
| ros | yes | sim, dashboard, rviz, bench, mission_control, report | none (internal: true) |

No `frontend` network; the stack publishes no host ports. Build-time egress
(apt/rosdep/Fuel models) occurs via `build.network: host` and is baked into the
image; it does not exist at runtime.

## Capability inventory (cross-cut)
No Linux capabilities granted to any service (all `cap_drop: [ALL]`, no `cap_add`).

## Privileged / host-namespace usage
`--privileged`, `network_mode: host`, `pid: host`, `userns_mode: host`: **none.**
`ipc: host`: **GUI services only** (`dashboard`, `rviz`, `mission_control`) for X11
MIT-SHM — see the waiver above; the headless `sim`/`bench`/`report` keep a private
IPC namespace. (The prior-art `drone_rescue_ws/docker-compose.yml` used host
networking and privileged; this stack drops both.)

## Host changes made during this dockerize run
Recorded for audit (these are host-level, outside the containers):
- Installed `linux-modules-nvidia-580-open-6.17.0-35-generic` (+ hwe metapackage)
  to provide the NVIDIA kernel module for the running kernel; loaded the modules.
- Installed `nvidia-container-toolkit` 1.19.1 from the NVIDIA apt repo and ran
  `nvidia-ctk runtime configure --runtime=docker` (writes `/etc/docker/daemon.json`),
  then restarted the Docker daemon.

Project source modified (by explicit user request, 2026-06-20):
- `drone_rescue_ws/src/drone_rescue_bringup/launch/multi_drone_simulation.launch.py`
  — the three `gz topic -e` primer subscribers now redirect stdout/stderr to
  `/dev/null` (subscription preserved, payload discarded). No security impact;
  removes a ~9 MB/s log flood. This is the only source change.

## Update cadence
- Image digest refresh: quarterly minimum, and on any critical CVE.
- This file regenerated on any topology change or dockerize re-run.
- Last review: 2026-06-19. Next due: 2026-09-19.
