#!/usr/bin/env bash

set -euo pipefail

PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-/home/br4c3/src/PX4-Autopilot}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500_mono_cam_down}"

if [[ ! -d "$PX4_AUTOPILOT_DIR" ]]; then
  echo "PX4-Autopilot directory not found: $PX4_AUTOPILOT_DIR" >&2
  echo "Set PX4_AUTOPILOT_DIR to your PX4-Autopilot checkout." >&2
  exit 1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "MicroXRCEAgent is not installed." >&2
  exit 1
fi

cleanup() {
  if [[ -n "${XRCE_PID:-}" ]]; then
    kill "$XRCE_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

MicroXRCEAgent udp4 -p 8888 &
XRCE_PID=$!

cd "$PX4_AUTOPILOT_DIR"
make px4_sitl "$PX4_SIM_MODEL"
