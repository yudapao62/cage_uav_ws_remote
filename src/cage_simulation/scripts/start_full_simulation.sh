#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_default="$(cd "$script_dir/../../.." && pwd)"
workspace="${CAGE_UAV_WS:-$workspace_default}"
ardupilot_dir="${ARDUPILOT_DIR:-/home/ubuntu/ardupilot}"
gazebo_plugin_dir="${ARDUPILOT_GAZEBO_DIR:-/home/ubuntu/ardupilot_gazebo}"
run_dir="${CAGE_SIM_RUN_DIR:-/tmp/cage_uav_sitl}"

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"

if [[ "${PERSISTENT_LATTICE_SIM:-0}" == "1" ]]; then
  deformation_preset="${DEFORMATION_PRESET:-medium}"
  deformation_seed="${DEFORMATION_SEED:-42}"
  for launch_arg in "$@"; do
    case "$launch_arg" in
      deformation_preset:=*) deformation_preset="${launch_arg#deformation_preset:=}" ;;
      deformation_seed:=*) deformation_seed="${launch_arg#deformation_seed:=}" ;;
    esac
  done
  /usr/bin/python3 "$workspace/src/cage_simulation/scripts/generate_multi_cage_world.py" \
    --fish-farm-config "$workspace/src/cage_simulation/config/fish_farm.yaml" \
    --layout-config "$workspace/src/cage_simulation/config/multi_cage_layout.yaml" \
    --models-dir "$workspace/src/cage_simulation/models" \
    --world-output "$workspace/src/cage_simulation/worlds/cage_mission_deformed.world" \
    --ground-truth-output "$workspace/src/cage_simulation/config/cage_lattice_ground_truth.json" \
    --deformation-preset "$deformation_preset" \
    --deformation-seed "$deformation_seed"
  exec roslaunch cage_simulation persistent_lattice_sim.launch "$@"
fi

# Jetson NX is ARM and cannot execute the x86_64 ArduPilot SITL binary from
# 90-3. Keep the same Cage ROS chain runnable with the ARM-safe mock FCU and
# Gazebo-only launch; real FCU/MAVROS remains the real-flight path.
if [[ "$(uname -m)" != "x86_64" || ! -x "$ardupilot_dir/build/sitl/bin/arducopter" || ! -f "$gazebo_plugin_dir/build/libArduPilotPlugin.so" ]]; then
  exec roslaunch cage_simulation simulation_nx3.launch "$@"
fi

for required in "$ardupilot_dir/build/sitl/bin/arducopter" "$gazebo_plugin_dir/build/libArduPilotPlugin.so" "$workspace/devel/setup.bash"; do
  test -e "$required" || { echo "Missing required artifact: $required" >&2; exit 1; }
done
mkdir -p "$run_dir"
export GAZEBO_MODEL_PATH="$workspace/src/cage_simulation/models:$gazebo_plugin_dir/models:/usr/share/gazebo-11/models"
# Keep the ArduPilot plugin first, while also exposing Gazebo ROS camera and
# Gazebo's built-in camera dependency.  Without the latter, Gazebo can parse
# the UAV sensor but libgazebo_ros_camera.so fails to load libCameraPlugin.so.
export GAZEBO_PLUGIN_PATH="$gazebo_plugin_dir/build:/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$gazebo_plugin_dir/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""
export DISABLE_ROS1_EOL_WARNINGS=1

# Avoid silently colliding with an older Gazebo master or GUI.  Stop the old
# process explicitly before retrying this script, so an apparently blank GUI
# cannot be mistaken for a newly loaded world.
if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
  echo "Gazebo is already running (gzserver/gzclient). Stop the old instance before restarting." >&2
  exit 1
fi

# Generate all visual-only scene assets from editable YAML sources.
/usr/bin/python3 "$workspace/src/cage_simulation/scripts/generate_fish_farm.py" \
  --config "$workspace/src/cage_simulation/config/fish_farm.yaml" \
  --output "$workspace/src/cage_simulation/models/fish_farm/model.sdf"
/usr/bin/python3 "$workspace/src/cage_simulation/scripts/generate_water_surface.py" \
  --config "$workspace/src/cage_simulation/config/water.yaml" \
  --output "$workspace/src/cage_simulation/models/water_surface/model.sdf"
/usr/bin/python3 "$workspace/src/cage_simulation/scripts/generate_uav_camera.py" \
  --config "$workspace/src/cage_simulation/config/camera.yaml" \
  --output "$workspace/src/cage_simulation/models/gimbal_small_2d/model.sdf"

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if rosnode list >/dev/null 2>&1; then
  echo "Using existing ROS master at ${ROS_MASTER_URI:-http://localhost:11311}"
else
  roscore & pids+=("$!")
  for _attempt in {1..20}; do
    rosnode list >/dev/null 2>&1 && break
    sleep 0.1
  done
fi

# A previous crash can leave /gazebo registered in rosmaster even though the
# process is gone.  That stale registration prevents the new launch from
# wiring MAVROS and the mission nodes reliably.  Only clean registrations
# that cannot be pinged; real running nodes are left untouched.
if rosnode list 2>/dev/null | grep -qx '/gazebo' && ! rosnode ping -c1 /gazebo >/dev/null 2>&1; then
  echo "Removing stale ROS node registrations before starting simulation"
  yes | rosnode cleanup >/dev/null 2>&1 || true
fi

gzserver --verbose -s libgazebo_ros_api_plugin.so \
  "$workspace/src/cage_simulation/worlds/cage_mission.world" & pids+=("$!")
sleep 2
if [[ "${START_GZCLIENT:-1}" != "0" && -n "${DISPLAY:-}" ]]; then
  gzclient --verbose & pids+=("$!")
else
  echo "Gazebo GUI disabled (set DISPLAY and START_GZCLIENT=1 to enable)"
fi
cd "$run_dir"
"$ardupilot_dir/build/sitl/bin/arducopter" -S --model gazebo-iris --speedup 1 --wipe \
  --defaults "$ardupilot_dir/Tools/autotest/default_params/copter.parm,$ardupilot_dir/Tools/autotest/default_params/gazebo-iris.parm,$workspace/src/cage_simulation/config/cage_sitl.parm" \
  --sim-address=127.0.0.1 -I0 --home=-35.363261,149.165230,584,353 & pids+=("$!")
sleep 2
roslaunch cage_bringup simulation.launch & pids+=("$!")
wait "${pids[-1]}"
