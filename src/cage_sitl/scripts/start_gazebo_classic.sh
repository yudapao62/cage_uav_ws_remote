#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash
exec roslaunch cage_sitl gazebo_classic_sitl.launch "$@"
