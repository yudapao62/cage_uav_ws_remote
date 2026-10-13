# Realflight 实验接口审计

审计日期：2026-09-20  
工程：`/home/nx3/cage_uav_ws`  
关联 GPS 工程：`/home/nx3/gps_ws`

## 1. 审计方法和边界

接口由源码、Launch/YAML、实际 `.msg` 定义交叉确认。审计时 NX3 上没有运行 ROS master，`rostopic list` 返回 `Unable to communicate with master`，因此运行时存在性、真实频率和现场设备插件必须在启动系统后执行 `scripts/check_experiment_topics.sh` 再确认。测试工具不会创建缺失话题。

Ground GPS 消息定义已在真实文件 `/home/nx3/gps_ws/src/gps_receiver/msg/AnchorFix.msg` 核实：

```text
int32 scene_id
string anchor_id
float64 lat
float64 lon
float64 altitude
bool valid
uint8 satellites
uint32 seq
string stamp
```

## 2. 已确认接口

| 层级 | 实际接口 | 类型 | 源码依据 | 方向/说明 |
|---|---|---|---|---|
| Ground GPS | `/farm/anchor_fix` | `gps_receiver/AnchorFix` | `cage_realflight/config/beacon_real.yaml`; `BeaconAdapter.__init__` | `beacon_gps_adapter` 订阅；A/B 共用话题，以 `anchor_id` 区分 |
| Ground GPS A raw | `/farm/scene_1/anchor_A/fix` | `gps_receiver/AnchorFix` | `gps_receiver/scripts/anchor_receiver.py:get_raw_anchor_pub()` | 三路 RViz 测试的 GPS_A |
| Ground GPS B raw | `/farm/scene_1/anchor_B/fix` | `gps_receiver/AnchorFix` | `gps_receiver/scripts/anchor_receiver.py:get_raw_anchor_pub()` | 三路 RViz 测试的 GPS_B |
| UAV GPS | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `MissionFSMNode.__init__`; `MavrosAdapter.__init__`; `GridNode.__init__` | MAVROS 发布，NX 订阅 |
| MAVROS namespace | `/mavros` | namespace | `MavrosAdapter(namespace='/mavros')` | 所有飞控接口的默认命名空间 |
| Planner 输出 | `/mission/waypoints` | `cage_msgs/MissionWaypointArray` | `PlannerNode.__init__`, `publish_plan()` | 完整规划航点数组，lat/lon + relative alt |
| 任务目标单元格 | `/feeding/current_cell` | `cage_msgs/CellTarget` | `FeedingTaskManager.__init__`, `on_mission()` | 到达航点后按 FSM 当前 cell 发布，不是连续控制 setpoint |
| Mission/FSM 状态 | `/mission/state` | `cage_msgs/MissionState` | `MissionFSMNode.__init__`, `publish_state()` | 包含 state、current_waypoint、current_cell_id、manual_override |
| FC mission 回读 | `/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | `MavrosAdapter.__init__`, `_on_waypoints()` | push 后调用 pull，MAVROS 发布飞控任务列表 |
| 航点到达 | `/mavros/mission/reached` | `mavros_msgs/WaypointReached` | `MissionFSMNode.__init__`, `on_reached()` | 飞控报告的到达序号 |
| 飞控状态/mode | `/mavros/state` | `mavros_msgs/State` | `MissionFSMNode.__init__`, `MavrosAdapter.__init__` | connected、armed、mode |
| RC 输入 | `/mavros/rc/in` | `mavros_msgs/RCIn` | `realflight_precheck.Precheck.__init__` | H12 经飞控/MAVROS 返回的通道值 |
| UAV local pose | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `trajectory_visualizer.py` | 可选运行时接口 |
| UAV local velocity | `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | MAVROS 标准插件；必须由运行时检查确认 | 可选记录，用于人工响应辅助判断 |

## 3. 控制架构结论

当前架构属于 **D：MAVLink Mission waypoint**。

`MissionFSMNode.tick()` 在 `UPLOAD_MISSION` 状态调用 `MavrosAdapter.upload_mission()`。该函数：

1. 调用 `/mavros/mission/clear`；
2. 将 `MissionWaypoint` 转为 `mavros_msgs/Waypoint`；
3. 默认在列表前插入一个 home 项；
4. 调用 `/mavros/mission/push`；
5. 调用 `/mavros/mission/pull`；
6. 将 `/mavros/mission/waypoints` 回读结果与上传列表比较；
7. 任务启动时调用 `/mavros/mission/set_current`，再调用 `/mavros/set_mode` 进入 `AUTO`。

没有发现项目向以下接口发布控制目标：

- `/mavros/setpoint_position/*`
- `/mavros/setpoint_raw/*`
- `/mavros/setpoint_velocity/*`

因此不存在可记录的“连续 MAVROS output setpoint topic”。实验二应把以下证据分开：

- NX 生成：`/mission/waypoints`
- NX 当前任务状态：`/mission/state`
- 当前投喂单元格报告：`/feeding/current_cell`
- FC/MAVROS 回读任务：`/mavros/mission/waypoints`
- FC 到点推进：`/mavros/mission/reached`
- 飞控最终采用情况：飞控日志 + mode/到点信息

ROS bag 不能记录 service 请求体。仅看到 planner topic 不能证明任务已上传，仅看到 mission 回读也不能证明飞控正在执行，必须结合上述多级证据。

## 4. 坐标和单位

- Planner 输入/输出：WGS84 latitude/longitude；任务高度为相对高度。
- MAVROS mission frame：`Waypoint.FRAME_GLOBAL_REL_ALT`。
- yaw：规划器通过 `degrees(atan2(...))` 生成 degree，写入 MAVLink waypoint `param4`。
- 网箱内部：以 UAV 稳定 GPS home 为参考的 ENU。
- 飞控 local position 的 ENU/NED 转换由 MAVROS 插件负责；现场应使用已发布消息的 frame_id 和飞控配置复核。
- Ground `AnchorFix.altitude` 的高度基准未在 `.msg` 中声明，不能未经设备文档确认就与 UAV GPS altitude 作垂直精度结论。

## 5. 配置文件

实验话题集中配置在 `config/experiment_topics.yaml`。任何现场话题变化应先修改该文件，然后运行检查脚本；分析器和录包脚本会读取同一份配置。
