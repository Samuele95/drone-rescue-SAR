#!/usr/bin/env bash
# Grant local containers access to the host X server for the `gui` / mission_control
# profiles. Run once per desktop session before `docker compose --profile gui up`.
# Scope: local unix-socket connections only (not network). Revoke with:
#   xhost -local:
set -euo pipefail
if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost not found (install x11-xserver-utils). On Wayland, GUIs use XWayland." >&2
  exit 1
fi
xhost +local: >/dev/null
echo "Granted: local containers may connect to X display ${DISPLAY:-:0}."
echo "Revoke after use with:  xhost -local:"
