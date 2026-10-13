#!/usr/bin/env bash
set -euo pipefail
ARDUPILOT_DIR="${ARDUPILOT_DIR:-/home/ubuntu/ardupilot}"
SITL_RUN_DIR="${SITL_RUN_DIR:-/tmp/cage_ardupilot_sitl}"
mkdir -p "$SITL_RUN_DIR"
exec /usr/bin/python3 "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter -f gazebo-iris --no-rebuild --no-mavproxy \
  --use-dir="$SITL_RUN_DIR" "$@"
