# K9 real flight integration

This package selects real vehicle configuration and connects an existing beacon
message to the shared grid, feeding task, planner, FSM and MAVROS adapter code.
It does not launch Gazebo, SITL, demo missions, simulation acceptance or the
feeder simulator. The simulation launch and configuration remain separate.

## Current site blockers

On `nx3`, the GPS workspace provides the verified interface:
`/farm/anchor_fix` with type `gps_receiver/AnchorFix`. Its A/B per-anchor
topics are `/farm/scene_1/anchor_A/fix` and `/farm/scene_1/anchor_B/fix`.
The message contains the required `scene_id`, `anchor_id`, `lat`, `lon`,
`altitude`, `valid`, `satellites`, `seq`, and string `stamp` fields. The real
flight configuration uses this aggregate topic and A/B IDs `A` and `B`.
If that package is moved, verify its replacement interface with:

```bash
rostopic list
rostopic type /actual/beacon/topic
rosmsg show actual_package/ActualMessage
```

Confirm A/B are the two real cage endpoints, measure rows, columns and cell
sizes, and set the configured offsets. The `site_alignment_confirmed` parameter
starts false and must only be set true after this measurement and RViz review.

## Setup and startup

Start the real beacon publisher and verify the message first. Start K9 and H12,
then MAVROS on the Jetson NX. Use the serial device and baud rate verified on
site. For initial integration, remove propellers. Load:

```bash
source /opt/ros/noetic/setup.bash
source /home/nx3/cage_uav_ws/devel/setup.bash
source /home/nx3/gps_ws/devel/setup.bash --extend
rosrun gps_receiver anchor_receiver.py
```

In another terminal start the real-flight chain:

```bash
source /opt/ros/noetic/setup.bash
source /home/nx3/cage_uav_ws/devel/setup.bash
source /home/nx3/gps_ws/devel/setup.bash --extend
roslaunch cage_realflight realflight.launch start_rviz:=true start_web:=true feeder_mode:=dry_run
```

`dry_run` permits preview and blocks Start Task. A real feeder driver must be
running, publishing fresh `/feeder/status`, before `feeder_mode:=real` may pass
the precheck. `auto_arm` is forced false. The pilot arms with H12 after reviewing
Preview. A manual mode change latches manual override in the shared FSM.

## RC9 feeder command

With MAVROS connected, start the real feeder driver by selecting real mode:

```bash
roslaunch cage_realflight realflight.launch feeder_mode:=real
```

Run the feeder once for `n` seconds from the NX (example: 5 seconds):

```bash
rosservice call /feeder/run_for_duration "duration_sec: 5.0"
```

The driver sends only RC channel 9 (`2000` while open), writes `1000` for one
second when the timer expires, and then releases channel 9 back to the H12.
The channel, PWM polarity, maximum duration and close-hold time are configured
in `config/feeding_real.yaml`. Verify `rc_pwm_on` and `rc_pwm_off` with the
propellers removed before enabling real mode.

`realflight_core.launch` runs without UI. `realflight_ui.launch` can be launched
separately. Run `scripts/record_realflight.sh` while the real ROS master is
active to save ten minute lz4 bag segments in `bags/realflight_YYYYMMDD_HHMMSS`.

## Geometry and safety

The adapter uses the Jetson receive time, validates both configured anchors,
checks satellite count, sequence, age, time skew, baseline and altitude
difference, then takes a median window. It rejects anchor jumps larger than
`max_anchor_jump_m`. The grid waits for a stable UAV GPS home and converts both
anchors into that same ENU frame. Preview and active task targets use a snapshot
of the grid model. The FSM invokes its feeder-off and RTL recovery chain if
valid beacon status is lost beyond the configured active mission timeout.

The beacon `stamp` is recorded verbatim and is not used for freshness. Beacon
altitude is not a feed altitude. The planner continues using relative mission
altitudes. No node in this package calls flight control services.

## K9 interface checklist (no propellers)

- [ ] `/mavros/state` shows K9 connected and H12 mode changes.
- [ ] `/mavros/global_position/global` gives stable, valid coordinates.
- [ ] `/mavros/rc/in` is fresh with channels populated.
- [ ] Mission clear, push, pull and reached are verified with K9.
- [ ] AUTO and RTL mode spelling and acceptance are verified.
- [ ] `/mavros/extended_state` and `/mavros/battery` are verified.
- [ ] Beacon A/B appear in Web and RViz at the measured cage endpoints.
- [ ] Preview produces the expected GPS waypoints for all three feeding modes,
      without arming or starting a mission.
- [ ] Beacon loss, sequence reset, jump rejection and geometry freeze are
      tested without propellers.
- [ ] The real feeder driver reports OFF/ON accurately; a simulator is absent.

Do not attempt a propeller flight before the above checks are complete.
