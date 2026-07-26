#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${JETSON_GCS_TOKEN:-}" ]]; then
  echo "Set JETSON_GCS_TOKEN before starting the gateway." >&2
  exit 1
fi

cd "$PROJECT_ROOT"
exec python3 bridge/jetson_gateway.py
