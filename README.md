# Cage UAV Scout & Feeding 仿真系统

这是一个基于 ROS Noetic、Gazebo Classic 和 Python 的网箱无人机任务仿真工作区。系统面向非完全规则、存在形变的水产养殖网箱，完成以下闭环：

```text
网页/REST 请求
  -> 无人机起飞并沿 Scout 侦察航线经过网箱
  -> 在线建立不规则网格地图
  -> 冻结、校验并持久化 Scout 地图
  -> 自动返航、降落、解除武装
  -> 重新加载历史地图
  -> 按网格单元执行 Feeding
  -> 自动返航和降落
```

当前仿真使用 `mock_mavros_node.py` 模拟飞控和 MAVROS 接口，持续发布 `/mavros/local_position/pose`。任务不会绕过位姿观测逻辑：无人机必须沿侦察路线进入并经过网箱区域，带噪声观测节点才会根据无人机位姿产生局部网格观测。

> 本 README 以已经在目标 Ubuntu 主机验证的仿真配置为准。真机接口位于 `cage_realflight`，但 K9 真机、真实相机、海况和人工接管仍需单独进行台架及海试验证。

## 主要功能

- Gazebo 中生成并显示带随机形变的非规则网箱；
- mock MAVROS 持续发布飞控、GPS、扩展状态和本地位姿；
- Web 页面一键启动或安全取消 Scout；
- 6 行蛇形侦察路线，默认 12 个 survey endpoints；
- Scout 在线地图构建、覆盖率统计、冻结和完整性校验；
- Scout 完成后自动 RTL、降落并解除武装；
- 每次 Scout 生成可审计、带 SHA-256 的任务产物；
- 历史 Scout 地图可列出、严格校验、显式激活并在重启后恢复；
- Feeding 支持全网格、行/列和指定 cell 三种模式；
- Feeding 全程锁定同一组地图身份和内容哈希；
- 地图、身份、心跳或 lease 不一致时 fail-closed，禁止开启 Feeder；
- RViz 显示网箱、航点、轨迹和任务状态；
- REST API、ROS topic/service 和命令行均可操作。

## 系统架构

```text
Gazebo irregular cage                 Web / REST API
         |                                   |
         v                                   v
mock MAVROS ------------------------ ScoutTaskManager
  /mavros/local_position/pose                |
         |                            Mission FSM
         v                                   |
pose-gated noisy observations <------ flight route
         |
         v
ScoutLatticeNode
  /scout/working_lattice_map
         |
         v
Scout artifact: map/result/quality/manifest
         |
         v
ScoutMapRegistryNode
  /scout/lattice_map + /scout/active_map_status
         |
         +-------------------------+
         |                         |
         v                         v
FeedingTaskManager        FeederIdentityNode
         |                         |
         +------------+------------+
                      v
                 Mission FSM
                      |
                      v
               Feeder ON / OFF
```

Scout 地图分为两个通道：

| Topic | 含义 |
|---|---|
| `/scout/working_lattice_map` | 当前 Scout 正在构建的可变地图 |
| `/scout/lattice_map` | Registry 发布的已冻结 active 地图，供 Feeding 使用 |
| `/scout/active_map_status` | active 地图身份、兼容性和 registry 心跳 |

新 Scout 开始时不会清除旧 active 地图。只有新任务完成、产物完整且通过严格校验后，才会原子替换 active 地图。

## 工作区结构

```text
src/
├── cage_simulation          # Gazebo、mock MAVROS、Scout 管理器和观测仿真
├── cage_grid                # 持久网格、关联、优化、地图仓库和 Registry
├── cage_waypoint_planner    # Scout 与 Feeding 航点规划
├── cage_mission_fsm         # 起飞、航线、RTL、降落和安全状态机
├── cage_mavros_adapter      # 隔离的 MAVROS 任务接口
├── cage_feeding_task        # Feeding 目标生成和任务管理
├── cage_feeder              # Feeder 控制及仿真
├── cage_web_bridge          # Web 页面和 REST API
├── cage_visualization       # RViz 可视化
├── cage_geo                 # WGS84 / ENU 坐标转换
├── cage_msgs                # ROS message 和 service 定义
├── cage_control_authority   # 人工/自动控制权管理
├── cage_bringup             # 系统配置、启动和验收工具
├── cage_sitl                # ArduPilot SITL 相关启动工具
└── cage_realflight          # 真机编排和实验接口
```

更完整的命令和故障处理见 [SIMULATION_COMMANDS_AND_USER_GUIDE.md](SIMULATION_COMMANDS_AND_USER_GUIDE.md)。

## 运行环境

已验证环境：

- Ubuntu 20.04；
- ROS Noetic；
- Gazebo Classic 11；
- Python 3；
- catkin workspace；
- 浏览器访问本机 Web 页面；
- 可选 RViz。

核心 ROS 依赖包括 `gazebo_ros`、`rviz`、`mavros`、`mavros_msgs`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`、`visualization_msgs`、`joy` 和 `rosbag`。

## 1. 获取代码

下面的路径是目标 Ubuntu 主机上的已验证路径：

```bash
git clone https://github.com/yudapao62/cage_uav_ws_remote.git \
  /home/ubuntu/cage_uav_ws
cd /home/ubuntu/cage_uav_ws
```

仓库当前为私有仓库，克隆时需要登录有权限的 GitHub 账号。

如果代码已经位于 `/home/ubuntu/cage_uav_ws`，跳过本节即可。

## 2. 安装依赖

先确保 ROS Noetic 和 Gazebo 11 已正确安装，然后执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ubuntu/cage_uav_ws

sudo apt update
sudo apt install -y \
  python3-rosdep \
  python3-numpy \
  python3-yaml \
  python3-geographiclib \
  python3-nose \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-rviz \
  ros-noetic-mavros \
  ros-noetic-joy

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

如果系统从未初始化过 rosdep，需要先执行一次：

```bash
sudo rosdep init
rosdep update
```

## 3. 编译工作区

目标机的用户级 Python 包与 Ubuntu 20.04 系统包可能存在版本冲突，因此构建和运行时必须保留 `PYTHONNOUSERSITE=1`：

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
catkin_make -j4
source /home/ubuntu/cage_uav_ws/devel/setup.bash
```

每次打开新终端都要执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash
export PYTHONNOUSERSITE=1
```

确认 ROS 能找到项目：

```bash
rospack find cage_simulation
```

预期输出：

```text
/home/ubuntu/cage_uav_ws/src/cage_simulation
```

## 4. 启动完整仿真

启动前确认没有另一套仿真正在运行：

```bash
pgrep -af 'roslaunch cage_simulation persistent_lattice_sim.launch'
pgrep -af gzserver
```

在 Ubuntu 桌面“终端 1”运行：

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
source /home/ubuntu/cage_uav_ws/devel/setup.bash

roslaunch cage_simulation persistent_lattice_sim.launch \
  deformation_preset:=medium \
  deformation_seed:=42 \
  observation_source:=oracle_noisy \
  feeder_visual_source:=oracle_noisy \
  scout_auto_start:=false \
  scout_record_bag:=true \
  start_gazebo:=true \
  start_rviz:=true \
  start_web:=true \
  web_port:=8081
```

不需要单独运行 `roscore`，`roslaunch` 会自动启动 ROS Master。

主要参数：

| 参数 | 含义 |
|---|---|
| `deformation_preset:=medium` | 使用 medium 网箱形变配置 |
| `deformation_seed:=42` | 固定形变随机种子，使场景可重复 |
| `observation_source:=oracle_noisy` | Scout 使用带噪声的位姿门控仿真观测 |
| `feeder_visual_source:=oracle_noisy` | Feeding 身份滤波使用带噪声视觉观测 |
| `scout_auto_start:=false` | 启动系统后等待网页或 API 请求，不自动起飞 |
| `scout_record_bag:=true` | 为每次 Scout 保存 ROS bag |
| `start_gazebo:=true` | 启动 Gazebo |
| `start_rviz:=true` | 启动 RViz |
| `start_web:=true` | 启动 Web 控制页面 |
| `web_port:=8081` | Web 监听端口 |

纯 SSH 或无桌面环境运行时，将 RViz 关闭：

```bash
roslaunch cage_simulation persistent_lattice_sim.launch \
  deformation_preset:=medium \
  deformation_seed:=42 \
  observation_source:=oracle_noisy \
  feeder_visual_source:=oracle_noisy \
  scout_auto_start:=false \
  scout_record_bag:=true \
  start_gazebo:=true \
  start_rviz:=false \
  start_web:=true \
  web_port:=8081
```

> 当前 world、ground-truth、`environment_id` 和几何 SHA-256 是配套的 `medium/42` 组合。不要只修改 preset 或 seed。更换 world 或 ground-truth 时，必须重新生成场景，并同步提供正确的 `environment_id` 和 `environment_geometry_sha256`。

## 5. 打开 Web 页面

在目标 Ubuntu 浏览器访问：

```text
http://127.0.0.1:8081/
```

也可以从终端打开：

```bash
xdg-open http://127.0.0.1:8081/
```

网页包含：

- Scout 开始、安全取消和进度；
- Scout 航点、覆盖率、地图 UUID/version、RTL、落地及解除武装状态；
- 历史地图列表、重新校验和激活；
- Feeding Preview、网格选择、任务启动和取消；
- 当前 active 地图和任务状态。

如果 Web 不在 8081 监听：

```bash
ss -ltnp | grep -E ':808[1-9]|:809[0-9]'
```

## 6. 检查仿真状态

保持终端 1 的 `roslaunch` 运行，在“终端 2”执行：

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash
export PYTHONNOUSERSITE=1
```

检查关键节点：

```bash
rosnode list | grep -E '^/(gazebo|mock_mavros|scout_lattice_node|scout_map_registry|scout_task_manager|feeding_task_manager_node|mission_fsm_node|cage_web_bridge)$' | sort
```

检查 Gazebo：

```bash
rosnode ping -c 1 /gazebo
```

检查 mock MAVROS 本地位姿：

```bash
rostopic hz /mavros/local_position/pose
```

正常频率约为 `20 Hz`。按 `Ctrl+C` 只会退出频率检查，不会停止仿真。

查看系统综合状态：

```bash
curl -sS http://127.0.0.1:8081/api/status | python3 -m json.tool
```

## 7. 启动 Scout 网箱探测

### 7.1 使用网页

1. 打开 `http://127.0.0.1:8081/`；
2. 目标网箱选择 `CAGE_0`；
3. 侦察高度保持默认 `25 m`；
4. 点击“开始 Scout 探测”；
5. 等待任务完成。无人机会自动返航、降落并解除武装。

正常状态顺序：

```text
PRECHECK -> PLANNING -> UPLOADING -> ARMING -> TAKEOFF
-> TRANSIT -> SCOUTING -> FINALIZING_MAP
-> RTL_RETURNING -> LANDING -> COMPLETED
```

### 7.2 使用命令行

每次 Scout 必须使用新的 `task_id`：

```bash
curl -sS -X POST \
  http://127.0.0.1:8081/api/scout/start \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "scout-demo-001",
    "cage_id": 0,
    "survey_relative_alt": 25.0,
    "auto_rtl": true,
    "auto_land": true
  }' | python3 -m json.tool
```

实时查看状态：

```bash
watch -n 1 'curl -sS http://127.0.0.1:8081/api/status | python3 -m json.tool'
```

也可以直接查看 ROS topic：

```bash
rostopic echo /scout/task_status
```

成功完成应满足：

```text
coverage: 1.0
map_finalized: True
map_ready_for_feeding: True
returned_home: True
landed: True
disarmed: True
```

### 7.3 安全取消 Scout

将 task ID 替换为正在运行的任务：

```bash
curl -sS -X POST \
  http://127.0.0.1:8081/api/scout/cancel \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"scout-demo-001"}' \
  | python3 -m json.tool
```

取消不会让无人机直接停在空中。系统会冻结部分地图、保存诊断产物，然后执行安全 RTL、降落和解除武装。取消产生的 partial map 不会替换旧 active 地图。

## 8. 查看和激活历史 Scout 地图

查看 active 地图：

```bash
rostopic echo -n 1 /scout/active_map_status
```

列出历史地图：

```bash
curl -sS \
  'http://127.0.0.1:8081/api/scout/maps?cage_id=0&include_invalid=true' \
  | python3 -m json.tool
```

查看当前 active：

```bash
curl -sS \
  http://127.0.0.1:8081/api/scout/maps/active \
  | python3 -m json.tool
```

新 Scout 成功后会通过同一套严格校验服务自动激活。需要手工切换历史地图时，从列表复制该地图的四项 pin，然后执行：

```bash
curl -sS -X POST \
  http://127.0.0.1:8081/api/scout/maps/activate \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "替换为历史 Scout task_id",
    "cage_id": 0,
    "expected_map_uuid": "替换为 map_uuid",
    "expected_map_version": 472,
    "expected_map_sha256": "替换为 map_sha256",
    "expected_manifest_sha256": "替换为 manifest_sha256"
  }' | python3 -m json.tool
```

地图由以下四项共同标识，不能只使用 version：

```text
map_uuid
map_version
map_sha256
manifest_sha256
```

## 9. Feeding Preview

Preview 只生成目标，不解锁、不起飞、不开启 Feeder。下面的命令选择 `R03C05` 和 `R03C06`：

```bash
curl -sS -X POST \
  http://127.0.0.1:8081/api/preview \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "feed-preview-001",
    "cage_id": 0,
    "mode": "SELECTED_CELLS",
    "rows": [],
    "cols": [],
    "selected_cell_ids": ["R03C05", "R03C06"],
    "repeat_count": 1,
    "feed_relative_alt": 15.0,
    "transit_relative_alt": 25.0
  }' | python3 -m json.tool
```

检查 Preview 结果和地图 pin：

```bash
rostopic echo -n 1 /feeding/task_status
rostopic echo -n 1 /feeding/target_cells
```

## 10. 启动 Feeding

确认 Preview 正确后执行：

```bash
curl -sS -X POST \
  http://127.0.0.1:8081/api/start \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "feed-run-001",
    "cage_id": 0,
    "mode": "SELECTED_CELLS",
    "rows": [],
    "cols": [],
    "selected_cell_ids": ["R03C05", "R03C06"],
    "repeat_count": 1,
    "feed_relative_alt": 15.0,
    "transit_relative_alt": 25.0
  }' | python3 -m json.tool
```

监控任务：

```bash
rostopic echo /feeding/task_status
rostopic echo /mission/state
rostopic echo /feeder/status
rostopic echo /feeder/identity_state
```

正常的两格任务行为：

```text
到达 R03C05 -> 身份确认 -> Feeder ON
离开 R03C05 -> Feeder OFF
到达 R03C06 -> 身份确认 -> Feeder ON
离开投喂区域 -> Feeder OFF
RTL -> LAND -> FINISHED -> TASK_COMPLETED
```

安全取消 Feeding：

```bash
curl -sS -X POST \
  http://127.0.0.1:8081/api/cancel \
  -H 'Content-Type: application/json' \
  -d '{}' | python3 -m json.tool
```

## 11. Feeding 目标模式

全网格：

```json
{
  "mode": "FULL_GRID",
  "rows": [],
  "cols": [],
  "selected_cell_ids": []
}
```

指定行：

```json
{
  "mode": "ROW_OR_COLUMN",
  "rows": [3],
  "cols": [],
  "selected_cell_ids": []
}
```

指定列：

```json
{
  "mode": "ROW_OR_COLUMN",
  "rows": [],
  "cols": [5],
  "selected_cell_ids": []
}
```

指定多个 cell：

```json
{
  "mode": "SELECTED_CELLS",
  "rows": [],
  "cols": [],
  "selected_cell_ids": ["R01C01", "R02C04", "R06C10"]
}
```

## 12. Scout 输出产物

每个 Scout 任务的输出目录：

```text
/home/ubuntu/.ros/cage_uav/scout_runs/<task_id>/
```

查看全部任务：

```bash
ls -lah /home/ubuntu/.ros/cage_uav/scout_runs
```

完整任务通常包含：

| 文件 | 内容 |
|---|---|
| `request.json` | 原始任务参数和请求身份 |
| `route.json` | 实际 Scout 航线和航点 |
| `events.jsonl` | 任务事件时间线 |
| `map.json` | 冻结后的持久网格地图 |
| `quality.json` | 覆盖率和地图质量指标 |
| `result.json` | 飞行、建图、RTL、落地等最终结果 |
| `manifest.json` | 文件清单、身份和 SHA-256 |
| `mission.bag` | 可选 ROS bag，供离线复盘 |

不要直接编辑完成任务目录中的文件。任何字节变化都会造成 manifest 哈希校验失败。

当前 active selector：

```text
/home/ubuntu/.ros/cage_uav/scout_runs/.active_map.json
```

不要手工编辑 selector；地图切换必须通过 Web 或 `/api/scout/maps/activate` 完成。

## 13. 验证重启恢复

重启前记录 active 地图：

```bash
rostopic echo -n 1 /scout/active_map_status
```

在运行 `roslaunch` 的终端按 `Ctrl+C`，等待节点全部退出，然后重新执行第 4 节启动命令。重启后再次运行：

```bash
rostopic echo -n 1 /scout/active_map_status
```

正常情况下 message 包含：

```text
Scout map restored from selector
```

重启前后的 UUID、version、map hash 和 manifest hash 应完全一致。

## 14. 运行测试

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
catkin_make -j4
catkin_make run_tests
catkin_test_results
```

持久网格专项验收：

```bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash
rosrun cage_bringup persistent_lattice_acceptance.py
```

## 15. 正确停止系统

在运行 `roslaunch` 的终端按 `Ctrl+C`，等待全部节点退出。

不要在 Scout、Feeding、RTL 或 LAND 过程中强制关闭 Gazebo。应先通过网页或 API 安全取消任务，等待无人机落地并解除武装后再停止系统。

## 常见问题

### 找不到 `persistent_lattice_sim.launch`

错误：

```text
RLException: [persistent_lattice_sim.launch] is neither a launch file...
```

解决：

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
catkin_make -j4
source /home/ubuntu/cage_uav_ws/devel/setup.bash
rospack find cage_simulation
```

### Web 页面打不开

```bash
rosnode list | grep cage_web_bridge
ss -ltnp | grep -E ':808[1-9]|:809[0-9]'
curl -v --max-time 5 http://127.0.0.1:8081/api/status
```

### `NO_ACTIVE_MAP` 或 `SCOUT_MAP_NOT_READY`

```bash
curl -sS \
  'http://127.0.0.1:8081/api/scout/maps?cage_id=0&include_invalid=true' \
  | python3 -m json.tool
```

先完成一次 Scout，或通过网页/API 激活一张 compatible 的历史地图。

### `REGISTRY_HEARTBEAT_STALE`

```bash
rosnode ping -c 1 /scout_map_registry
rostopic hz /scout/active_map_status
```

如果 registry 不存在，应安全停止并重启完整 launch，不要绕过 heartbeat 检查。

### `MAP_HASH_MISMATCH`

重新列出历史地图，复制同一条记录的 UUID、version、map SHA-256 和 manifest SHA-256。不要手工猜测或只使用 version。

### `MAP_LEASE_CONFLICT` 或 `MAP_SWITCH_BUSY`

当前 Feeding 任务持有地图 lease，不能切换 active 地图。等待任务完成，或使用 `/api/cancel` 安全取消 Feeding。

### `MISSION_BUSY`、`SCOUT_BUSY` 或 `FEEDING_BUSY`

```bash
rostopic echo -n 1 /mission/state
rostopic echo -n 1 /scout/task_status
rostopic echo -n 1 /feeding/task_status
```

等待当前任务进入终态后再提交下一任务。

### 重复启动 Gazebo

```bash
pgrep -af 'roslaunch cage_simulation persistent_lattice_sim.launch'
pgrep -af gzserver
```

如果已经有输出，请回到原来的 `roslaunch` 终端继续使用，不要启动第二套系统。

## 设计与验收文档

- [SCOUT_FEEDER_IRREGULAR_LATTICE_SIMULATION_PLAN.md](SCOUT_FEEDER_IRREGULAR_LATTICE_SIMULATION_PLAN.md)
- [PERSISTENT_LATTICE_IMPLEMENTATION_REPORT.md](PERSISTENT_LATTICE_IMPLEMENTATION_REPORT.md)
- [SCOUT_WEB_MISSION_IMPLEMENTATION_PLAN.md](SCOUT_WEB_MISSION_IMPLEMENTATION_PLAN.md)
- [SCOUT_WEB_MISSION_IMPLEMENTATION_REPORT.md](SCOUT_WEB_MISSION_IMPLEMENTATION_REPORT.md)
- [SCOUT_MAP_RELOAD_FEEDING_PERSISTENCE_PLAN.md](SCOUT_MAP_RELOAD_FEEDING_PERSISTENCE_PLAN.md)
- [SCOUT_MAP_RELOAD_FEEDING_PERSISTENCE_VALIDATION.md](SCOUT_MAP_RELOAD_FEEDING_PERSISTENCE_VALIDATION.md)
- [SIMULATION_COMMANDS_AND_USER_GUIDE.md](SIMULATION_COMMANDS_AND_USER_GUIDE.md)

