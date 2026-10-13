#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="${EXPERIMENT_CONFIG:-$EXP_DIR/config/experiment_topics.yaml}"
RATE_WINDOW_SEC="${RATE_WINDOW_SEC:-3}"

command -v rostopic >/dev/null || { echo "rostopic not found; source ROS first" >&2; exit 2; }
rostopic list >/dev/null 2>&1 || { echo "ROS master is unavailable" >&2; exit 2; }

cfg() {
  python3 - "$CONFIG_FILE" "$1" "$2" <<'PY'
import sys,yaml
c=yaml.safe_load(open(sys.argv[1],encoding='utf-8'))
v=c.get(sys.argv[2],{}).get(sys.argv[3])
print('' if v is None else v)
PY
}

show_topic() {
  local label="$1" key="$2" topic type rate
  topic="$(cfg topics "$key")"
  printf '\n[%s]\n' "$label"
  if [[ -z "$topic" ]] || ! rostopic list | grep -Fxq "$topic"; then
    echo "[NOT FOUND]"
    [[ -n "$topic" ]] && echo "configured topic: $topic"
    return
  fi
  type="$(rostopic type "$topic" 2>/dev/null || true)"
  rate="$(timeout "$RATE_WINDOW_SEC" rostopic hz -w 3 "$topic" 2>/dev/null | awk '/average rate:/{v=$3} END{print v}')" || true
  echo "topic: $topic"
  echo "type: ${type:-[UNKNOWN]}"
  echo "rate: ${rate:-[NO SAMPLE]}"
}

show_service() {
  local label="$1" key="$2" service type
  service="$(cfg services "$key")"
  printf '\n[%s]\n' "$label"
  if [[ -z "$service" ]] || ! rosservice list | grep -Fxq "$service"; then
    echo "[NOT FOUND]"
    [[ -n "$service" ]] && echo "configured service: $service"
    return
  fi
  type="$(rosservice type "$service" 2>/dev/null || true)"
  echo "service: $service"
  echo "type: ${type:-[UNKNOWN]}"
}

show_topic "GROUND GPS" ground_gps
show_topic "GROUND GPS A" ground_gps_a
show_topic "GROUND GPS B" ground_gps_b
show_topic "UAV GPS" uav_gps
show_topic "PLANNER PATH" planner_path
show_topic "CURRENT SETPOINT" current_target
show_topic "MAVROS OUTPUT SETPOINT" __continuous_setpoint_not_used
echo "architecture: MAVLink Mission upload; no continuous setpoint topic is used"
show_topic "FC MISSION ECHO" mavros_mission_waypoints
show_topic "MAVROS STATE" mavros_state
show_topic "RC INPUT" rc_input
show_topic "FSM STATE" fsm_state
show_topic "UAV LOCAL POSITION" local_position
show_topic "UAV VELOCITY" local_velocity

show_service "MISSION PUSH" mission_push
show_service "MISSION PULL" mission_pull
show_service "MISSION SET CURRENT" mission_set_current
show_service "SET MODE" set_mode
show_service "ARMING" arming
