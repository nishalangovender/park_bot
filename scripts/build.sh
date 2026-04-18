#!/usr/bin/env bash
# Build the park_bot colcon workspace.
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon not found. Source ROS2 Humble and re-run." >&2
  exit 1
fi

colcon build --symlink-install "$@"
echo
echo "✓ build complete — source install/setup.bash before running."
