#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash
OUTPUT="${1:-manual_takeover_$(date +%Y%m%d_%H%M%S)}"
exec rosbag record -O "$OUTPUT" \
  /mavros/state \
  /mavros/rc/in \
  /mavros/rc/out \
  /mavros/rc/override \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/global_position/global \
  /mavros/imu/data \
  /mavros/mission/waypoints \
  /joy \
  /control/owner \
  /control/rc_override_released \
  /manual/takeover_request \
  /manual/land_request \
  /mission/state \
  /mission/waypoints \
  /feeding/current_cell
