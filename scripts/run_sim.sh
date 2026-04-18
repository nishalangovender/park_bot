#!/usr/bin/env bash
# Launch the full park_bot simulation: Ignition Gazebo, ROS2 Control, SLAM, NAV2, RViz.
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

if [[ ! -f install/setup.bash ]]; then
  echo "error: workspace not built. Run scripts/build.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source install/setup.bash

# Launch in background so we can also bring up RViz from the same shell.
ros2 launch park_bot sim.launch.py &
SIM_PID=$!

trap 'kill "$SIM_PID" 2>/dev/null || true' EXIT

rviz2 -d src/park_bot/config/main.rviz
