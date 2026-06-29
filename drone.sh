#!/usr/bin/env bash
# ============================================================================
# drone.sh: one-command manager for the dockerized Drone-Rescue SAR simulation
#
#   ./drone.sh             open the interactive TUI menu (also: tui)
#   ./drone.sh deps        check host dependencies (docker, compose, NVIDIA, X11)
#   ./drone.sh install     build the container image (runs deps first)
#   ./drone.sh run [TGT]   run it (TGT: sim|gui=mission-control|dashboard|bench|report|stop|status)
#   ./drone.sh delete      tear the project down (containers + volume + image)
#   ./drone.sh readme      show the README
#   ./drone.sh tutorial    show the step-by-step TUTORIAL
#   ./drone.sh help        this help
#
# Everything runs through Docker Compose: no host ROS/Gazebo install needed.
# ============================================================================
set -euo pipefail

# --- resolve project root (this script's dir) so it works from anywhere ------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

IMAGE="${DRONE_IMAGE:-drone-rescue-sim:dockerize}"
PROJECT="drone-rescue"                 # compose project name (see compose.yaml `name:`)
RUNS_VOLUME="${PROJECT}_drone-runs"
DOCS_DIR="$SCRIPT_DIR/dockerize"
COMPOSE=(docker compose)               # Compose v2

# --- pretty output -----------------------------------------------------------
if [ -t 1 ]; then
  B=$'\033[1m'; G=$'\033[32m'; R=$'\033[31m'; Y=$'\033[33m'; C=$'\033[36m'; N=$'\033[0m'
else B=""; G=""; R=""; Y=""; C=""; N=""; fi
ok()   { printf '  %s✓%s %s\n' "$G" "$N" "$*"; }
bad()  { printf '  %s✗%s %s\n' "$R" "$N" "$*"; }
warn() { printf '  %s!%s %s\n' "$Y" "$N" "$*"; }
info() { printf '%s==>%s %s\n' "$C" "$N" "$*"; }
die()  { printf '%sError:%s %s\n' "$R" "$N" "$*" >&2; exit 1; }

have() { command -v "$1" >/dev/null 2>&1; }

# The project is "installed" when its container image (the build artifact) exists.
is_installed() { docker image inspect "$IMAGE" >/dev/null 2>&1; }

# ============================================================================
# deps: verify host prerequisites
# ============================================================================
cmd_deps() {
  info "Host dependency check"
  local hard_fail=0

  # Docker engine
  if have docker && docker version >/dev/null 2>&1; then
    ok "Docker engine: $(docker version --format '{{.Server.Version}}' 2>/dev/null || echo present)"
  else bad "Docker engine not available (install Docker, ensure the daemon runs)"; hard_fail=1; fi

  # Compose v2
  if docker compose version >/dev/null 2>&1; then
    ok "Docker Compose: $(docker compose version --short 2>/dev/null)"
  else bad "Docker Compose v2 plugin missing (need 'docker compose')"; hard_fail=1; fi

  # NVIDIA host driver (optional: GPU render; software fallback otherwise)
  if have nvidia-smi && nvidia-smi -L >/dev/null 2>&1; then
    ok "NVIDIA driver: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -1)"
  else warn "NVIDIA driver not active — GPU render unavailable; sim falls back to software (slower)."; fi

  # nvidia-container-toolkit (optional: needed for --gpus)
  if have nvidia-ctk; then
    ok "nvidia-container-toolkit: $(nvidia-ctk --version 2>/dev/null | head -1 | awk '{print $NF}')"
  else warn "nvidia-container-toolkit not found — GPU passthrough disabled."; fi

  # GPU reaches containers (only if both above present)
  if have nvidia-smi && nvidia-smi -L >/dev/null 2>&1 && have nvidia-ctk; then
    if docker run --rm --gpus all "$IMAGE" nvidia-smi -L >/dev/null 2>&1; then
      ok "GPU visible inside containers (--gpus all works)"
    elif docker run --rm --gpus all nvidia/cuda:13.0.0-base-ubuntu24.04 nvidia-smi -L >/dev/null 2>&1; then
      ok "GPU visible inside containers (verified via cuda base image)"
    else warn "GPU not reaching containers — check 'nvidia-ctk runtime configure --runtime=docker' + restart docker."; fi
  fi

  # X11 display (optional: only the GUI profile needs it)
  if [ -S /tmp/.X11-unix/X0 ] || [ -n "${DISPLAY:-}" ]; then
    ok "X11 display available (DISPLAY=${DISPLAY:-unset}) — GUI profile usable after 'xhost +local:'"
  else warn "No X11 display — headless works; GUI profile won't show windows."; fi

  # Disk headroom (image ~5.6G + logs in RAM)
  local freeG; freeG=$(df -BG --output=avail "$SCRIPT_DIR" 2>/dev/null | tail -1 | tr -dc '0-9')
  if [ -n "${freeG:-}" ] && [ "$freeG" -ge 10 ]; then ok "Disk free: ${freeG} GB"
  else warn "Low disk free (${freeG:-?} GB); image needs ~6 GB."; fi

  # Image built?
  if docker image inspect "$IMAGE" >/dev/null 2>&1; then
    ok "Image built: $IMAGE ($(docker image ls "$IMAGE" --format '{{.Size}}' 2>/dev/null | head -1))"
  else warn "Image not built yet — run './drone.sh install'."; fi

  echo
  if [ "$hard_fail" -ne 0 ]; then
    die "Required dependencies missing (see ✗ above). Docker + Compose v2 are mandatory."
  fi
  ok "All required dependencies satisfied."
}

# ============================================================================
# install: build the image
# ============================================================================
cmd_install() {
  info "Checking dependencies before build"
  cmd_deps || true
  echo
  info "Building $IMAGE (ROS 2 Jazzy + Gazebo Harmonic + colcon + Fuel models)"
  warn "First build downloads/compiles a lot — expect 15-25 min. Re-builds are cached."
  "${COMPOSE[@]}" build
  echo
  ok "Build complete."
  echo "Next:  ./drone.sh run        # start the headless simulation"
  echo "       ./drone.sh tutorial   # guided walk-through"
}

# ============================================================================
# run: start / operate the project
# ============================================================================
_require_image() {
  docker image inspect "$IMAGE" >/dev/null 2>&1 || die "Image not built. Run './drone.sh install' first."
}

# Force a clean slate before every launch: remove ALL containers belonging to
# this Compose project: both the long-running services AND the one-off
# `compose run` containers (mission_control / bench), which `compose down` does
# NOT remove. Force-removal also defeats any `restart: unless-stopped` policy, so
# a crashed sim or a leftover dashboard can't keep coming back. Net effect: every
# `run` is a fresh run with no previous mission still alive.
_cleanup_all() {
  info "Closing any previous run (containers, GUIs, missions)"
  local ids
  ids=$(docker ps -aq --filter "label=com.docker.compose.project=${PROJECT}" 2>/dev/null || true)
  if [ -n "$ids" ]; then
    docker rm -f $ids >/dev/null 2>&1 || true
    ok "Removed $(printf '%s\n' "$ids" | grep -c .) previous container(s)"
  fi
  "${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true
}

_wait_healthy() {  # $1 = container name, $2 = timeout seconds
  local name="$1" timeout="${2:-200}" i=0
  while [ "$i" -lt "$timeout" ]; do
    local st rs
    st=$(docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$name" 2>/dev/null || echo gone)
    rs=$(docker inspect --format '{{.State.Status}}' "$name" 2>/dev/null || echo gone)
    [ "$st" = healthy ] && { ok "$name is healthy"; return 0; }
    [ "$rs" != running ] && { bad "$name is $rs"; docker compose logs --tail=30 sim || true; return 1; }
    sleep 5; i=$((i+5))
  done
  warn "$name did not report healthy within ${timeout}s; check './drone.sh run status'"
  return 1
}

# Export the host's render/video group IDs so the non-root container user can open
# /dev/dri for GUI hardware rendering (compose x-gui-gl reads these). Harmless if
# the groups don't exist: the compose defaults and Mesa's software fallback apply.
_export_gpu_gids() {
  local r v
  r="$(getent group render 2>/dev/null | cut -d: -f3)"; export RENDER_GID="${r:-992}"
  v="$(getent group video  2>/dev/null | cut -d: -f3)"; export VIDEO_GID="${v:-44}"
}

# Pick the X display the container should use. A stale/wrong $DISPLAY (e.g. :0 when
# the live server is :1) makes the GUI talk to a non-existent X server and GL fails.
# Honour $DISPLAY only if its socket exists; otherwise use the first live socket.
_export_display() {
  local cur="${DISPLAY:-}" num
  if [ -n "$cur" ]; then
    num="${cur#*:}"; num="${num%%.*}"
    [ -S "/tmp/.X11-unix/X${num}" ] && { export DISPLAY=":${num}"; return; }
  fi
  local s
  for s in /tmp/.X11-unix/X*; do
    [ -e "$s" ] && { export DISPLAY=":${s##*/X}"; ok "Using X display $DISPLAY"; return; }
  done
  warn "No X socket in /tmp/.X11-unix — GUI needs a running X/XWayland server."
}

_ensure_reports_dir() {
  # Host folder for exported PDF reports (bind-mounted to /reports in the
  # report/mission_control containers). World-writable so the non-root
  # container user (UID 10001) can write through the mount.
  local d="${DRONE_REPORTS_DIR:-$SCRIPT_DIR/reports}"
  mkdir -p "$d" 2>/dev/null && chmod 777 "$d" 2>/dev/null || true
  export DRONE_REPORTS_DIR="$d"
}

cmd_run() {
  _require_image
  _ensure_reports_dir
  local target="${1:-sim}"; shift || true
  case "$target" in
    sim|"")
      _cleanup_all
      info "Starting headless simulation (NVIDIA GL if available)"
      "${COMPOSE[@]}" up -d
      _wait_healthy drone-rescue-sim 200
      "${COMPOSE[@]}" ps
      echo "Logs:   ./drone.sh run logs     |   Stop: ./drone.sh run stop" ;;
    gui|mc|mission-control)
      # The GUI is the mission *selection* launcher: Mission Control's Setup tab,
      # where you pick a scenario and start a mission (it spawns its own sim).
      _cleanup_all; _export_gpu_gids; _export_display
      info "Opening Mission Control — pick a scenario and launch a mission"
      [ -x "$DOCS_DIR/xhost-grant.sh" ] && bash "$DOCS_DIR/xhost-grant.sh" || warn "xhost helper missing; run 'xhost +local:' manually."
      "${COMPOSE[@]}" --profile tools run --rm mission_control ;;
    dashboard|monitor|live)
      # The live operator Dashboard (monitors a running mission) + RViz.
      _cleanup_all; _export_gpu_gids; _export_display
      info "Granting local X access + starting a FRESH sim with the live Dashboard + RViz"
      [ -x "$DOCS_DIR/xhost-grant.sh" ] && bash "$DOCS_DIR/xhost-grant.sh" || warn "xhost helper missing; run 'xhost +local:' manually."
      "${COMPOSE[@]}" --profile gui up -d
      _wait_healthy drone-rescue-sim 200
      "${COMPOSE[@]}" ps ;;
    bench)
      _cleanup_all
      info "Running headless benchmark sweep (software render, reproducible)"
      if [ "$#" -gt 0 ]; then
        "${COMPOSE[@]}" --profile tools run --rm bench "$@"
      else
        warn "No args given — using defaults: spiral_out / default / 1 trial"
        "${COMPOSE[@]}" --profile tools run --rm bench \
          --patterns spiral_out --scenarios default --trials 1 \
          --wall-budget 3600 --runs-dir /data/runs/drone_sh_bench
      fi ;;
    report)
      info "Running the PDF report generator"
      "${COMPOSE[@]}" --profile tools run --rm report "${@:---help}" ;;
    stop)
      info "Stopping all services (volumes kept)"
      "${COMPOSE[@]}" --profile gui --profile tools stop 2>/dev/null || "${COMPOSE[@]}" stop
      "${COMPOSE[@]}" down --remove-orphans
      ok "Stopped." ;;
    status|ps)
      "${COMPOSE[@]}" ps ;;
    logs)
      "${COMPOSE[@]}" logs -f --tail=100 sim ;;
    *)
      die "Unknown run target '$target'. Try: sim | gui | dashboard | bench | report | stop | status | logs" ;;
  esac
}

# ============================================================================
# delete: tear everything down (DESTRUCTIVE)
# ============================================================================
cmd_delete() {
  local assume_yes=0
  [ "${1:-}" = "-y" ] || [ "${1:-}" = "--yes" ] && assume_yes=1
  warn "This removes: all containers, the '${RUNS_VOLUME}' volume (recorded runs!), and the image '$IMAGE'."
  if [ "$assume_yes" -ne 1 ]; then
    printf "%sType 'delete' to confirm:%s " "$Y" "$N"; read -r reply
    [ "$reply" = "delete" ] || die "Aborted."
  fi
  info "Stopping and removing containers + volumes"
  "${COMPOSE[@]}" --profile gui --profile tools down -v --remove-orphans 2>/dev/null || "${COMPOSE[@]}" down -v --remove-orphans
  info "Removing image $IMAGE"
  docker image rm "$IMAGE" 2>/dev/null && ok "Image removed" || warn "Image not present"
  info "Pruning dangling build layers"
  docker builder prune -f >/dev/null 2>&1 || true
  ok "Project deleted. (Source tree untouched; rebuild with './drone.sh install'.)"
}

# ============================================================================
# readme / tutorial: show bundled docs
# ============================================================================
_show_doc() {  # $1 = file
  local f="$1"
  [ -f "$f" ] || die "Doc not found: $f"
  if [ -t 1 ] && have glow; then glow -p "$f"
  elif [ -t 1 ] && have less; then less -R "$f"
  else cat "$f"; fi
}
cmd_readme()   { _show_doc "$DOCS_DIR/README.md"; }
cmd_tutorial() { _show_doc "$DOCS_DIR/TUTORIAL.md"; }

# ============================================================================
# tui: interactive menu over the commands above
# ============================================================================
_tui_header() {
  printf '\033[H\033[2J'                    # clear screen, cursor home
  local inst run gpu
  if is_installed; then inst="${G}installed${N}"; else inst="${R}not installed${N}"; fi
  if [ -n "$(docker compose ps -q sim 2>/dev/null)" ]; then run="${G}running${N}"; else run="${Y}stopped${N}"; fi
  if have nvidia-smi && nvidia-smi -L >/dev/null 2>&1; then gpu="${G}NVIDIA${N}"; else gpu="${Y}software${N}"; fi
  printf '%s' "$B"
  printf '============================================================\n'
  printf '   Drone-Rescue  ·  control panel\n'
  printf '============================================================%s\n' "$N"
  printf '   project: %b   sim: %b   render: %b\n' "$inst" "$run" "$gpu"
  printf '%s------------------------------------------------------------%s\n' "$C" "$N"
}

cmd_tui() {
  [ -t 0 ] || die "The TUI needs an interactive terminal. Use './drone.sh help' for the command list."
  local choice installed
  while true; do
    if is_installed; then installed=1; else installed=0; fi
    _tui_header
    printf '  %sSetup%s\n'   "$B" "$N"
    printf '    1) Check dependencies\n'
    # Install is offered only when the project is NOT yet installed.
    [ "$installed" -eq 0 ] && printf '    2) Install / build image\n'
    printf '  %sRun%s\n'     "$B" "$N"
    printf '    3) Start headless simulation\n'
    printf '    4) Open GUI — Mission Control (select & launch a mission)\n'
    printf '    5) Run benchmark  (defaults)\n'
    printf '    6) Live Dashboard + RViz  (monitor a running mission)\n'
    printf '    7) Generate report\n'
    printf '  %sInspect%s\n' "$B" "$N"
    printf '    8) Status\n'
    printf '    9) Follow logs  (Ctrl-C to return)\n'
    printf '   10) Stop the stack\n'
    printf '  %sDocs%s\n'    "$B" "$N"
    printf '   11) Read README\n'
    printf '   12) Read TUTORIAL\n'
    # Delete is offered only when the project IS installed.
    if [ "$installed" -eq 1 ]; then
      printf '  %sDanger%s\n'  "$R" "$N"
      printf '   13) Delete everything\n'
    fi
    printf '    0) Quit\n'
    if [ "$installed" -eq 0 ]; then
      printf '\n  %s(not installed — choose 2 to install)%s\n' "$Y" "$N"
    fi
    printf '\n  %sSelect:%s ' "$C" "$N"
    read -r choice || break
    echo
    case "$choice" in
      1)  cmd_deps      || true ;;
      2)  if [ "$installed" -eq 1 ]; then
            warn "Already installed. Choose 13 (Delete) first if you want to rebuild from scratch."
          else cmd_install || true; fi ;;
      3)  cmd_run sim   || true ;;
      4)  cmd_run gui       || true ;;
      5)  cmd_run bench     || true ;;
      6)  cmd_run dashboard || true ;;
      7)  cmd_run report --help || true ;;
      8)  cmd_run status || true ;;
      9)  cmd_run logs  || true ;;     # Ctrl-C ends the follow and returns here
      10) cmd_run stop  || true ;;
      11) cmd_readme    || true ;;
      12) cmd_tutorial  || true ;;
      13) if [ "$installed" -eq 1 ]; then cmd_delete || true
          else warn "Not installed — nothing to delete."; fi ;;
      0|q|quit|exit) printf 'Bye.\n'; break ;;
      "") continue ;;
      *) warn "Unknown option: $choice" ;;
    esac
    case "$choice" in 0|q|quit|exit) ;; *) printf '\n  %sPress Enter to return to the menu…%s' "$C" "$N"; read -r _ || break ;; esac
  done
}

# ============================================================================
# help / dispatch
# ============================================================================
cmd_help() {
  sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# Bare invocation in an interactive terminal opens the TUI; non-interactive or
# explicit commands behave as a normal CLI.
main_cmd="${1:-}"
if [ -z "$main_cmd" ]; then
  if [ -t 0 ] && [ -t 1 ]; then main_cmd="tui"; else main_cmd="help"; fi
fi

case "$main_cmd" in
  tui|menu)              cmd_tui ;;
  deps|check|check-deps) shift; cmd_deps "$@" ;;
  install|build)         shift; cmd_install "$@" ;;
  run|start)             shift; cmd_run "$@" ;;
  delete|destroy|rm)     shift; cmd_delete "$@" ;;
  readme)                shift; cmd_readme "$@" ;;
  tutorial)              shift; cmd_tutorial "$@" ;;
  help|-h|--help)        cmd_help ;;
  *) die "Unknown command '${main_cmd}'. Run './drone.sh help'." ;;
esac
