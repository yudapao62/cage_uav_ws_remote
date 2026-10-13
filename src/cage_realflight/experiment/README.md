# Realflight 三项实机实验工具

这些工具只读取、记录和分析现有 ROS 数据，不发布控制命令，不修改 planner、FSM、GPS、飞控参数或 setpoint。

## 1. 每个终端先加载环境

```bash
source /opt/ros/noetic/setup.bash
source /home/nx3/cage_uav_ws/devel/setup.bash
source /home/nx3/gps_ws/devel/setup.bash --extend
cd /home/nx3/cage_uav_ws/src/cage_realflight/experiment
```

启动真实系统后先检查接口：

```bash
./scripts/check_experiment_topics.sh
```

脚本逐项显示 topic、消息类型和采样频率。MAVROS OUTPUT SETPOINT 显示 `[NOT FOUND]` 是当前 Mission 架构的预期结果；同时应看到 FC MISSION ECHO 和 mission service。

## 2. 实验一：Ground GPS 与 UAV GPS

先设置天线机械偏移。定义是 **Ground 相对 UAV** 的 ENU 米值：

```bash
export GROUND_ANCHOR_ID=A
export ANTENNA_DX_M=0.00
export ANTENNA_DY_M=0.00
export ANTENNA_DZ_M=0.00
```

录制 10 分钟：

```bash
./scripts/record_gps_test.sh GPS_TEST_01 600
```

省略时长时按一次 Ctrl+C 正常结束：

```bash
./scripts/record_gps_test.sh GPS_TEST_01
```

离线分析：

```bash
./scripts/gps_compare_analyzer.py results/gps/GPS_TEST_01
```

输出 `gps_report.md`、`gps_metrics.csv` 和 6 张图。依次执行 `GPS_TEST_02`、`GPS_TEST_03`，不要覆盖已有目录。

## 3. 实验二：航点链路

第一阶段必须拆桨或保持 disarmed/dry_run。先开始录包，再通过现有 Web/ROS 操作产生预览或任务：

```bash
./scripts/record_waypoint_test.sh WP_01
```

完成后按 Ctrl+C，分析：

```bash
./scripts/waypoint_chain_analyzer.py results/waypoint/WP_01
```

若一个 bag 中存在多个任务，可指定 mission ID：

```bash
./scripts/waypoint_chain_analyzer.py results/waypoint/WP_01 --mission-id <MISSION_ID>
```

输出 `waypoint_report.md`、`waypoint_metrics.csv` 和 ENU 俯视图。报告分别标注 NX planner、当前 cell 报告、FC mission echo 和到点证据的边界。

## 4. 实验三：H12 人工接管

先从 Level 0 仿真和 Level 1 无桨实机开始。配置中的 `rc_mode_channel: 5` 只是当前待现场确认值；先观察：

```bash
rostopic echo /mavros/rc/in
```

切换 H12 mode 开关，确认变化通道后修改 `config/experiment_topics.yaml`。录制：

```bash
./scripts/record_override_test.sh RC_04
```

切入人工模式后保持 30 秒以上，再结束录制。首次分析不要宣称飞机已响应：

```bash
./scripts/manual_override_analyzer.py results/override/RC_04
```

结合现场观察/视频确认飞机确实响应人工操纵后：

```bash
./scripts/manual_override_analyzer.py results/override/RC_04 --manual-response-confirmed
```

输出 `override_report.md`、`override_metrics.csv` 和统一时间轴图。

## 5. 结果目录

每次录制生成：

```text
results/<gps|waypoint|override>/<TEST_NAME>/
├── raw.bag
├── bag_info.yaml
├── metadata.yaml
├── notes.txt
├── *_metrics.csv
├── *_report.md
└── figures/
```

脚本拒绝覆盖已有 `raw.bag`。分析器只读原始 bag，可重复生成报告。

## 6. 可选配置

- 使用其他配置文件：`export EXPERIMENT_CONFIG=/path/to/topics.yaml`
- 使用其他结果根目录：`export EXPERIMENT_RESULTS_ROOT=/data/realflight_results`
- Topic 频率采样秒数：`RATE_WINDOW_SEC=5 ./scripts/check_experiment_topics.sh`
- GPS 配对阈值：分析命令加 `--max-sync-delta 0.3`

## 7. 已知边界

- Ground `AnchorFix.stamp` 是字符串，分析统一使用 rosbag receipt time。
- Mission 通过 ROS service 上传，service 请求体不在 rosbag 中；FC 回读 mission list 和飞控日志共同作为证据。
- Ground 与 UAV 高度基准尚未确认，垂直偏差不能直接作为定位精度结论。
- `WAIT_ARM` 阶段存在 NX 周期性重新请求 AUTO 的风险窗口，详见 `MANUAL_OVERRIDE_AUDIT.md`。

## 8. 三路 GPS 实时 RViz

Ground A、Ground B 与 UAV GPS 的实时 ENU、距离、同步状态、累计统计、散点轨迹和录制方法见 `README_GPS_RVIZ_TEST.md`。

```bash
roslaunch cage_realflight gps_relative_test.launch
cd /home/nx3/cage_uav_ws/src/cage_realflight/experiment
./scripts/record_gps_rviz_test.sh GPS_TEST_01 600
```
