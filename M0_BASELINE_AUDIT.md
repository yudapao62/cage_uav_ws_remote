# M0 Baseline Audit

Date: 2026-10-08

## Local source snapshot

- Workspace: `/Users/yudapao/cage_uav_ws_remote`
- Source files (excluding `__pycache__`): 188
- Deterministic aggregate SHA-256: `0cf064fdff3d08967255da567907f4492bd64561f1c3a31819646f315e32646a`
- Python: 3.9.6
- Git metadata: absent

## Remote comparison

Target attempted: `ubuntu@192.168.1.233:/home/ubuntu/cage_uav_ws/src`

Result: SSH port 22 timed out on 2026-10-08. No remote files were read or modified. Remote comparison remains a hard gate before deployment or synchronization back to the target host.

## Existing regression baseline

The tests were run directly with the source package directories on `PYTHONPATH`.

Passed:

- `cage_geo`: 4
- `cage_grid`: 6
- `cage_mission_fsm`: 10
- `cage_visualization`: 3
- `cage_waypoint_planner`: 6
- Total directly runnable tests: 29

Not runnable in this unbuilt macOS snapshot:

- `cage_mavros_adapter/test/test_adapter.py`: generated `cage_msgs` Python package unavailable.
- `cage_realflight/test/test_beacon_adapter.py`: ROS `sensor_msgs` Python package unavailable.

These are environment blocks, not recorded test failures. They must run after `catkin_make` in the ROS Noetic target environment.

## Baseline capability conclusion

- Gazebo scene, UAV camera, regular grid, feeding modes, planner, FSM, MAVROS adapter, Web and RViz are present.
- The current scene jitter is visual only; task geometry remains a uniform affine grid.
- No Scout image subscriber, persistent lattice, coordinate covariance, Feeder adjacent-cell filter or vision-v9 ROS adapter exists in this snapshot.
- Local SCOPE v9 artifacts are unrelated to the Scout/Feeder cage-identity pipeline.

