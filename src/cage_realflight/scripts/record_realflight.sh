#!/usr/bin/env bash
set -euo pipefail
workspace="${CAGE_UAV_WS:-/home/ubuntu/cage_uav_ws}"
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
beacon_topic_a="$(rosparam get /beacon/input_topic_a)"
beacon_topic_b="$(rosparam get /beacon/input_topic_b)"
if [[ -z "$beacon_topic_a" || "$beacon_topic_a" == *TODO* ||
      -z "$beacon_topic_b" || "$beacon_topic_b" == *TODO* ]]; then
  echo 'Configure both real beacon topics before recording.' >&2
  exit 1
fi
out="$workspace/bags/realflight_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
exec rosbag record --lz4 --split --duration=10m -O "$out/realflight" \
  /mavros/state /mavros/global_position/global /mavros/global_position/rel_alt \
  /mavros/mission/waypoints /mavros/mission/reached /mavros/extended_state /mavros/battery \
  "$beacon_topic_a" "$beacon_topic_b" /cage/beacon/status /cage/geometry_live /cage/geometry_validated /cage/grid_info \
  /feeding/task_request /feeding/task_status /feeding/current_cell \
  /mission/waypoints /mission/state /feeder/command /feeder/status
