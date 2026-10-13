# 三路 GPS 实时 ENU / RViz 测试

该功能是独立只读测试节点。它不发布飞控命令，不修改 GPS、planner、FSM、MAVROS mission 或 setpoint。节点退出不会影响飞行控制。

## 已确认的三路输入

| 角色 | 设备 | Topic | 类型 | 区分方式 |
|---|---|---|---|---|
| GPS_A | 地面锚点 A | `/farm/scene_1/anchor_A/fix` | `gps_receiver/AnchorFix` | `anchor_id == A` |
| GPS_B | 地面锚点 B | `/farm/scene_1/anchor_B/fix` | `gps_receiver/AnchorFix` | `anchor_id == B` |
| GPS_C | UAV 飞控 GPS | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | 独立 topic |

Ground 消息字段为 `lat/lon/altitude/valid/satellites/seq/stamp`；UAV 消息使用 `latitude/longitude/altitude/status/header.stamp`，`NavSatFix` 本身没有卫星数量字段。

所有坐标通过项目已有的 `cage_geo.geodesy.LocalENU` 做 WGS84/ECEF/ENU 转换。RViz 中：

```text
X = East
Y = North
Z = Up
```

默认 `flatten_z=true`，点和连线画在水平面，但标签和 JSON/CSV 保留真实 Up 与高度差。

## 启动前

```bash
source /opt/ros/noetic/setup.bash
source /home/nx3/cage_uav_ws/devel/setup.bash
source /home/nx3/gps_ws/devel/setup.bash --extend
```

先启动 Ground GPS 和 MAVROS，确认：

```bash
rostopic hz /farm/scene_1/anchor_A/fix
rostopic hz /farm/scene_1/anchor_B/fix
rostopic hz /mavros/global_position/global
```

## 启动可视化

```bash
roslaunch cage_realflight gps_relative_test.launch
```

只启动节点、不打开 RViz：

```bash
roslaunch cage_realflight gps_relative_test.launch start_rviz:=false
```

使用固定 ENU 原点时，修改 `config/gps_relative_visualizer.yaml`：

```yaml
reference:
  mode: fixed
  latitude: 你的纬度
  longitude: 你的经度
  altitude: 你的高度
```

默认 `first_valid` 会把第一帧有效 GPS_A 固定为原点，之后不会随 GPS_A 漂移而移动原点。

## RViz 内容

- 红球：Ground GPS A；绿球：Ground GPS B；蓝球：UAV GPS C；
- 灰球：数据 INVALID 或超过 2 秒未更新；
- 点旁文字：ENU、状态、卫星数、累计 East/North 均值和标准差；
- 黄线：A-B、A-C、B-C；
- 线中点文字：水平距离、ΔE、ΔN、ΔH、距离均值/标准差和同步差；
- 紫色箭头：A → C 实时误差矢量；
- 红/绿/蓝坐标轴：East/North/Up；
- 散点：每路最近 1000 个有效位置；
- 总状态面板：三路有效性和三组水平距离。

当任一路 INVALID/STALE 时，相关距离显示 `N/A`，不会继续把旧距离显示成有效值。三路使用 ROS receipt time；消息时间差超过 0.2 秒时显示 `SYNC WARNING`，且该组数据不进入累计距离统计。

调试 JSON：

```bash
rostopic echo /gps_test/relative_status
```

## Ground Truth

在 `config/gps_relative_visualizer.yaml` 中启用某一组实测基线：

```yaml
ground_truth:
  ab: {enable: true, distance: 10.000}
```

RViz 会显示 GPS 水平距离、Ground Truth 以及二者差值。

## 录制 rosbag 和 CSV

进入实验目录：

```bash
cd /home/nx3/cage_uav_ws/src/cage_realflight/experiment
```

记录 10 分钟：

```bash
./scripts/record_gps_rviz_test.sh GPS_TEST_01 600
```

或手动停止：

```bash
./scripts/record_gps_rviz_test.sh GPS_TEST_01
```

按一次 Ctrl+C 后脚本会完成 bag 索引和 metadata。结果：

```text
results/gps_rviz/GPS_TEST_01/
├── raw.bag
├── bag_info.yaml
├── metadata.yaml
├── notes.txt
├── gps_relative.csv
└── figures/
```

录包包含：

- `/farm/scene_1/anchor_A/fix`；
- `/farm/scene_1/anchor_B/fix`；
- `/mavros/global_position/global`；
- `/gps_test/relative_status`；
- `/gps_test/markers`、`/gps_test/trails`；
- `/tf`、`/tf_static`；
- 可用时附加 `/mavros/state`、`/cage/beacon/status`。

若不想录 Marker：

```bash
RECORD_GPS_RVIZ_MARKERS=0 ./scripts/record_gps_rviz_test.sh GPS_TEST_01 600
```

`gps_relative.csv` 由可视化节点直接写入，包含三路原始经纬高、三路 ENU、三组水平距离、三组 ΔE/ΔN/ΔU、各路 receipt timestamp 和有效状态。

## 现场建议顺序

1. 三路数据源运行并通过 `check_experiment_topics.sh`；
2. 启动 `gps_relative_test.launch`；
3. 确认 A/B/C 均显示 VALID；
4. 把需要对比的天线放到同一点并记录实际机械偏移；
5. 启动录包，保持静止 10–20 分钟；
6. 观察 A-C 的距离、ΔE、ΔN、累计均值/标准差和散点范围；
7. 实验结束后保留 `raw.bag`，不要用 CSV 替代原始数据。
