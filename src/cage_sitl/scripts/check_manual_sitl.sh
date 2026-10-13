#!/usr/bin/env bash
set -u
source /opt/ros/noetic/setup.bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash

show_topic() {
  local label="$1" topic="$2"
  echo "[$label]"
  local topic_type
  topic_type="$(rostopic type "$topic" 2>/dev/null || true)"
  echo "type: ${topic_type:-MISSING}"
  if [[ -n "$topic_type" ]]; then
    rostopic info "$topic" 2>/dev/null | sed -n '1,14p'
    timeout 3 rostopic hz --wall-time -w 10 "$topic" 2>/dev/null | tail -1 || true
    timeout 3 rostopic echo -n 1 "$topic" 2>/dev/null | sed -n '1,28p' || true
  fi
}

echo '[ARDUPILOT]'
pgrep -af '/arducopter' || echo 'running: NO'
show_topic MAVROS /mavros/state
show_topic JOYSTICK /joy
show_topic 'RC OVERRIDE' /mavros/rc/override
show_topic 'RC INPUT' /mavros/rc/in
show_topic 'CONTROL OWNER' /control/owner
show_topic 'MISSION FSM' /mission/state
