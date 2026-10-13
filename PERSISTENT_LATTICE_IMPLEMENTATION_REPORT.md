# Scout-Feeder 不规则持久格网实施报告

日期：2026-10-09

依据：`SCOUT_FEEDER_IRREGULAR_LATTICE_SIMULATION_PLAN.md`

## 已完成

- M1：共享顶点非规则网箱、mild/medium/strong、独立多网箱 seed、SDF/GT 同源和几何校验。
- M2：隔离 `/sim_gt/cage/lattice`、无全局 ID 的因果 oracle-noisy 局部观测、噪声/漏检接口。
- M3：整数偏移 Top-K、identity/visibility 双状态、开放边界 birth、漏检恢复、歧义冻结、版本化地图及显式 JSON 保存/恢复。
- M4：名义/绝对/相对约束、Huber 鲁棒优化、坐标协方差和过期膨胀。
- M5：GPS 马氏候选、局部邻域视觉似然、任务转移先验、re-entry、四态拒判、任务地图快照和投料硬门禁。
- M7：5 个独立形变网箱、RViz/Web 状态、JSONL/rosbag 记录入口和批量评测脚本。
- 仿真运动闭环：`mock_mavros_node.py` 以有限速度执行标准 MAVROS 航点，持续发布 GPS、ENU pose 和 velocity，并同步 Gazebo `iris_demo`；`scout_route_commander.py` 自动上传 12 段蛇形 Scout 航线。
- 有限 FoV birth：从未进入过预期 FoV 的相邻格允许渐进式边界扩展；已被预期但漏检的内部格需要多帧证据，避免一次误检直接创建身份。

默认仿真资产：

- world：`src/cage_simulation/worlds/cage_mission_deformed.world`
- aggregate GT：`src/cage_simulation/config/cage_lattice_ground_truth.json`
- cage 数量：5，每个 60 cells；各 cage 使用独立 seed。
- aggregate SHA-256：`c38519d3594e1757c4f9cb1821b38d04bba34dd9aa25a2c511f18e9da4cb65c1`

## 自动验收

命令：

```bash
python3 src/cage_bringup/scripts/persistent_lattice_acceptance.py \
  --scenario-set all \
  --seeds 101,102,103,104,105 \
  --output-dir /tmp/cage_lattice_acceptance_final
```

覆盖 mild、medium、strong 共 15 个场景：

| 指标 | 结果 |
|---|---:|
| Scout committed associations | 675 |
| Identity accuracy | 100% |
| Slip | 0 |
| Nominal RMSE | 2.4264 m |
| Estimated RMSE | 0.1194 m |
| 坐标改善 | 95.08% |
| Feeder candidate recall | 100% |
| Feeder wrong commit | 0 |
| Ambiguous recall | 100% |
| 歧义/丢失/地图过期时投喂触发 | 0 |

同一脚本同时输出第 12.4 节要求的独立 baseline/ablation 记录：逐帧重编号、名义格网、普通 track ID、GPS 最近邻、单格外观、去掉 task prior、去掉坐标协方差、去掉 abstention；这些记录不混入完整方法的通过指标。

本机可运行的 66 个纯 Python 回归测试全部通过。`cage_mavros_adapter` 和 `cage_realflight` 的两个 ROS 消息依赖测试需要在 ROS Noetic catkin build 后运行。

## 运行

构建后，默认资产可直接启动：

```bash
source devel/setup.bash
roslaunch cage_simulation persistent_lattice_sim.launch \
  deformation_preset:=medium \
  deformation_seed:=42 \
  observation_source:=oracle_noisy \
  feeder_visual_source:=oracle_noisy \
  start_scout_route:=true \
  start_rviz:=true \
  start_web:=true
```

该 launch 默认启动 headless `gzserver`。需要 Gazebo Classic 图形窗口时，在远端桌面另开终端运行：

```bash
export DISPLAY=:0
source /opt/ros/noetic/setup.bash
source ~/cage_uav_ws/devel/setup.bash
gzclient --verbose
```

如需在启动前按参数重新生成全部网箱资产：

```bash
PERSISTENT_LATTICE_SIM=1 \
src/cage_simulation/scripts/start_full_simulation.sh \
  deformation_preset:=strong deformation_seed:=77 \
  observation_source:=oracle_noisy feeder_visual_source:=oracle_noisy \
  start_rviz:=true start_web:=true
```

批量生成 rosbag 时增加：

```text
record_bag:=true bag_output:=/tmp/cage_persistent_lattice.bag
```

## 90-3 在线闭环验证

已通过 Tailscale SSH 登录 `ubuntu@90-3`，目标工作区为 `/home/ubuntu/cage_uav_ws`。同步前已备份远端源码，随后完成 Noetic `catkin_make`、launch 展开和真实 Gazebo 运行验证。

2026-10-09 使用 `medium/seed=42/oracle_noisy` 的在线结果：

| 项目 | 结果 |
|---|---:|
| Scout 航线 | 12/12 航点连续执行完成 |
| 局部位姿 | `/mavros/local_position/pose` 持续发布 |
| 持久地图 | 60/60 cells confirmed |
| Missing cell | 0 |
| 在线 nominal RMSE | 2.5683 m |
| 在线 estimated RMSE | 0.0392 m |
| 在线坐标改善 | 98.47% |
| ROS pose 与 Gazebo model 位置差 | 0.0001 m |
| ROS/Gazebo 运行错误 | 0 |

验证结束时 `gzserver`、`gzclient`、RViz、Web 和完整 ROS launch 均保持运行。由于目标机 8080 已被其他服务占用，Web bridge 本次监听 `http://100.94.162.41:8081`。

## 尚未闭环的项目

- M6 vision-v9：尚未把目标机上的真实 v9 checkpoint/输出协议接入 `LocalLatticeObservation`。当前在线闭环使用 `oracle_noisy`，没有伪造真实模型结果。
