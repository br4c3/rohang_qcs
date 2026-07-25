#!/usr/bin/env bash

set -euo pipefail

PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-/home/br4c3/src/PX4-Autopilot}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500_mono_cam_down}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRAY_ARCHIVE="${TRAY_ARCHIVE:-$PROJECT_ROOT/survivor_tray_obj.zip}"
GENERATED_MODEL_ROOT="$PROJECT_ROOT/.generated/gz_models"
TRAY_MODEL_SDF="$GENERATED_MODEL_ROOT/survivor_tray_obj/model.sdf"
# Gazebo is ENU (X=east, Y=north), while PX4 local position is NED.
# Place the tray 5 m north of home, matching the generated landing plan.
TRAY_X="${TRAY_X:-0.0}"
TRAY_Y="${TRAY_Y:-5.0}"
TRAY_Z="${TRAY_Z:-0.04}"

if [[ ! -d "$PX4_AUTOPILOT_DIR" ]]; then
  echo "PX4-Autopilot directory not found: $PX4_AUTOPILOT_DIR" >&2
  echo "Set PX4_AUTOPILOT_DIR to your PX4-Autopilot checkout." >&2
  exit 1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "MicroXRCEAgent is not installed." >&2
  exit 1
fi

if [[ -f "$TRAY_ARCHIVE" ]]; then
  if ! command -v unzip >/dev/null 2>&1; then
    echo "unzip is required to load the Gazebo tray model." >&2
    exit 1
  fi
  mkdir -p "$GENERATED_MODEL_ROOT"
  unzip -oq "$TRAY_ARCHIVE" -d "$GENERATED_MODEL_ROOT"
  export GZ_SIM_RESOURCE_PATH="$GENERATED_MODEL_ROOT:${GZ_SIM_RESOURCE_PATH:-}"
fi

cleanup() {
  if [[ -n "${XRCE_PID:-}" ]]; then
    kill "$XRCE_PID" 2>/dev/null || true
  fi
  if [[ -n "${TRAY_SPAWNER_PID:-}" ]]; then
    kill "$TRAY_SPAWNER_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

MicroXRCEAgent udp4 -p 8888 &
XRCE_PID=$!

spawn_tray() {
  if [[ ! -f "$TRAY_MODEL_SDF" ]]; then
    return
  fi

  for _ in $(seq 1 60); do
    if gz service -l 2>/dev/null | grep -q "^/world/default/create$"; then
      gz service \
        -s /world/default/create \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "sdf_filename: \"$TRAY_MODEL_SDF\", name: \"survivor_tray\", pose: { position: { x: $TRAY_X, y: $TRAY_Y, z: $TRAY_Z } }"
      return
    fi
    sleep 1
  done

  echo "Gazebo tray spawn timed out." >&2
}

spawn_tray &
TRAY_SPAWNER_PID=$!

cd "$PX4_AUTOPILOT_DIR"
make px4_sitl "$PX4_SIM_MODEL"
