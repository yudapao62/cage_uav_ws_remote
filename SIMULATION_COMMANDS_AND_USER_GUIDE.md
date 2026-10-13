# 网箱 UAV 仿真系统命令行与使用指南

本文适用于直接在目标电脑 `90-3` 的 Ubuntu 桌面和终端中操作，不需要 SSH，也不需要配置端口转发。

工作区固定为：

```text
/home/ubuntu/cage_uav_ws
```

当前经过验收的环境组合为：

```text
deformation_preset = medium
deformation_seed   = 42
observation_source = oracle_noisy
feeder_visual_source = oracle_noisy
```

## 1. 系统功能概览

完整仿真包含以下部分：

- Gazebo：不规则网箱和无人机仿真；
- mock MAVROS：持续发布飞控、GPS 和 `/mavros/local_position/pose`；
- Scout：执行网箱侦察航线、建立持久地图、返航和自动降落；
- Scout Map Registry：严格验证、激活和重启恢复历史地图；
- Feeding：根据当前 active Scout 地图生成投喂目标并执行投喂；
- Mission FSM：统一控制起飞、航线、Feeder、RTL 和降落；
- Web：通过浏览器执行 Scout、历史地图激活、Preview 和 Feeding；
- RViz：显示网箱、无人机轨迹、航点和状态。

地图分成两个独立通道：

```text
/scout/working_lattice_map  Scout 正在构建的可变地图
/scout/lattice_map          Registry 发布的已冻结 active 地图
```

Feeding 只能使用 `/scout/lattice_map`，新 Scout 开始时不会清除或覆盖旧 active 地图。只有新 Scout 成功完成、产物完整并通过严格校验后，才会自动替换 active 地图。

## 2. 打开终端和进入工作区

在 Ubuntu 桌面打开终端，执行：

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
```

`PYTHONNOUSERSITE=1` 必须保留，用于避开用户目录中不兼容的 `setuptools/importlib_metadata`。

## 3. 首次编译或源码修改后重新编译

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
catkin_make -j4
source /home/ubuntu/cage_uav_ws/devel/setup.bash
```

成功时末尾应看到：

```text
Built target cage_msgs_generate_messages
```

以后每个新终端都要执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash
export PYTHONNOUSERSITE=1
```

可以把它理解为“让当前终端认识本工作区的 ROS 包、消息和服务”。

## 4. 检查是否已经有仿真在运行

启动前先执行：

```bash
pgrep -af 'roslaunch cage_simulation persistent_lattice_sim.launch'
```

如果有输出，说明完整仿真已经运行，不要再次启动第二套 ROS/Gazebo。

也可以检查：

```bash
rosnode list 2>/dev/null | grep -E '^/(gazebo|scout_map_registry|mission_fsm_node)$'
```

## 5. 启动完整仿真

建议在“终端 1”以前台方式运行，便于查看日志和用 `Ctrl+C` 安全停止：

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

主要参数含义：

| 参数 | 含义 |
|---|---|
| `deformation_preset:=medium` | 使用 medium 不规则网箱形变配置 |
| `deformation_seed:=42` | 固定形变随机种子，保证可重复 |
| `observation_source:=oracle_noisy` | Scout 使用带噪声的仿真观测 |
| `feeder_visual_source:=oracle_noisy` | Feeding 身份滤波使用带噪声视觉观测 |
| `scout_auto_start:=false` | 启动系统后不自动开始 Scout，等待 Web/API 命令 |
| `scout_record_bag:=true` | Scout 任务保存 ROS bag |
| `start_gazebo:=true` | 启动 Gazebo |
| `start_rviz:=true` | 启动 RViz |
| `start_web:=true` | 启动 Web 控制页面 |
| `web_port:=8081` | Web 首选监听端口 |

如果更换 world、ground-truth、preset 或 seed，必须同步提供正确的 environment ID 和几何文件 SHA-256。不能把 medium/42 的环境身份直接用于另一套网箱。

## 6. 打开 Web 页面

在目标 Ubuntu 的浏览器打开：

```text
http://127.0.0.1:8081/
```

也可以用命令打开：

```bash
xdg-open http://127.0.0.1:8081/
```

如果 8081 被占用，Web 节点会尝试后续端口。检查实际监听端口：

```bash
ss -ltnp | grep web_bridge
```

## 7. 打开监控终端

保持终端 1 中的 `roslaunch` 运行，再打开“终端 2”：

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

检查无人机本地位姿发布频率：

```bash
rostopic hz /mavros/local_position/pose
```

正常值约为：

```text
average rate: 20.000
```

按 `Ctrl+C` 退出频率检查，不会停止仿真。

## 8. 推荐的正常使用流程

推荐按以下顺序操作：

```text
启动系统
  -> 查看或激活一张 Scout 地图
  -> Feeding Preview
  -> 确认目标和地图 pin
  -> 启动 Feeding
  -> 自动 RTL、降落
```

需要重新侦察时：

```text
启动新 Scout
  -> 旧 active 地图继续存在
  -> 无人机沿 Scout 航线穿过网箱区域
  -> 完成建图
  -> 自动 RTL、降落、解除武装
  -> 生成 schema v2 artifact
  -> Registry 严格校验
  -> 校验成功后自动激活新地图
```

## 9. 查看当前 active Scout 地图

```bash
rostopic echo -n 1 /scout/active_map_status
```

只查看核心字段：

```bash
rostopic echo -n 1 /scout/active_map_status | \
grep -E '^(status|task_id|map_uuid|map_version|map_sha256|manifest_sha256|coverage|frozen|compatible|message):'
```

有效 active 地图应满足：

```text
status: 1
coverage: 1.0
frozen: True
compatible: True
```

检查 active map 唯一发布者：

```bash
rostopic info /scout/lattice_map
```

预期唯一发布者：

```text
/scout_map_registry
```

## 10. 列出全部历史 Scout 地图

目标机没有预装 `jq`，统一使用 Python 格式化 JSON：

```bash
curl -s \
  'http://127.0.0.1:8081/api/scout/maps?cage_id=0&include_invalid=true' \
  | python3 -m json.tool
```

查看当前 active：

```bash
curl -s \
  'http://127.0.0.1:8081/api/scout/maps/active' \
  | python3 -m json.tool
```

Web 页面中的“历史 Scout 地图”表格也能完成相同操作。只有满足以下条件的地图才能激活：

- Scout 任务结果为 `COMPLETED`；
- mapping outcome 为 `SUCCESS`；
- 地图为 frozen；
- coverage 达标；
- manifest 中所有文件 SHA-256 正确；
- map/result/quality/manifest 身份一致；
- frame、anchor、reference 和 environment 与当前运行环境兼容；
- 当前没有运行中的 Scout、Feeding、Mission 或有效 Feeding map lease。

## 11. 激活已验收的历史 v1 地图

```bash
curl -s -X POST \
  'http://127.0.0.1:8081/api/scout/maps/activate' \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "scout-web-1791526104020",
    "cage_id": 0,
    "expected_map_uuid": "7dedb190-3d6e-4d0d-b01c-5cb04ce549c0",
    "expected_map_version": 472,
    "expected_map_sha256": "9507a9383674937ce3cc73770401400656172d3c0552f2a8a8d653cb2ae628e9",
    "expected_manifest_sha256": "83e021d50cdc3c69842b335dc749396deeb146ec7ee61bae1ac508c414c11de2"
  }' \
  | python3 -m json.tool
```

这是 legacy v1 仿真地图，Web 会显示 reference/environment 来自管理员兼容 profile 的警告。它已通过本项目仿真验收，但不应用于真实飞行。

## 12. 激活已验收的 schema v2 地图

```bash
curl -s -X POST \
  'http://127.0.0.1:8081/api/scout/maps/activate' \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "scout-v2-validation-001",
    "cage_id": 0,
    "expected_map_uuid": "a942d5cf-0306-44d5-a956-858c23fb8b00",
    "expected_map_version": 472,
    "expected_map_sha256": "02e01ef3bb2202fb0f4dc275c2850a27e8c1f7a1c2fbe89758872e9f5785013e",
    "expected_manifest_sha256": "a7ae2a8cb57ca1df4a12e5c66f309d4c3695a1ce79351a5f9e07d9e19f66255a"
  }' \
  | python3 -m json.tool
```

不能只填写 version。系统使用下面四项共同标识地图：

```text
map_uuid
map_version
map_sha256
manifest_sha256
```

两张地图即使 version 一样，只要 UUID 或任一 hash 不同，也不会互相冒充。

## 13. 从命令行启动新 Scout

每次必须使用从未使用过的新 `task_id`：

```bash
curl -s -X POST \
  'http://127.0.0.1:8081/api/scout/start' \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "scout-new-001",
    "cage_id": 0,
    "survey_relative_alt": 25.0,
    "auto_rtl": true,
    "auto_land": true
  }' \
  | python3 -m json.tool
```

监控 Scout：

```bash
rostopic echo /scout/task_status
```

常见状态顺序：

```text
PRECHECK
PLANNING
UPLOADING
ARMING
TAKEOFF
TRANSIT
SCOUTING
FINALIZING_MAP
RTL_RETURNING
LANDING
COMPLETED
```

关键完成条件：

```text
coverage: 1.0
map_finalized: True
map_ready_for_feeding: True
returned_home: True
landed: True
disarmed: True
```

工作地图监控：

```bash
rostopic echo /scout/working_lattice_map
```

Scout 运行期间检查 active 地图仍未被清空：

```bash
rostopic echo -n 1 /scout/active_map_status
```

## 14. 安全取消 Scout

把 `task_id` 改成正在运行的 Scout 任务：

```bash
curl -s -X POST \
  'http://127.0.0.1:8081/api/scout/cancel' \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "scout-new-001"
  }' \
  | python3 -m json.tool
```

取消不是直接停机。系统会关闭建图会话、进入安全恢复、RTL、降落并解除武装。取消或失败产生的 partial map 不会自动替换旧 active 地图。

## 15. Feeding Preview：只规划，不起飞

Preview 两个指定网格：

```bash
curl -s -X POST \
  'http://127.0.0.1:8081/api/preview' \
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
  }' \
  | python3 -m json.tool
```

检查 Preview 状态：

```bash
rostopic echo -n 1 /feeding/task_status
```

检查目标坐标和地图 pin：

```bash
rostopic echo -n 1 /feeding/target_cells
```

预期：

```text
task_state: TASK_PREVIEW
total_targets: 2
targets: R03C05, R03C06
map_uuid/map_version/map_sha256/manifest_sha256 与 active map 完全一致
```

Preview 不会解锁、起飞或开启 Feeder。

## 16. 启动完整 Feeding

确认 Preview 正确后执行：

```bash
curl -s -X POST \
  'http://127.0.0.1:8081/api/start' \
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
  }' \
  | python3 -m json.tool
```

监控 Feeding：

```bash
rostopic echo /feeding/task_status
```

监控 Mission FSM：

```bash
rostopic echo /mission/state
```

监控 Feeder：

```bash
rostopic echo /feeder/status
```

监控身份滤波：

```bash
rostopic echo /feeder/identity_state
```

两格任务的正常行为：

```text
到达 R03C05，身份一致，Feeder 开启
离开 R03C05，Feeder 关闭
到达 R03C06，身份一致，Feeder 开启
离开投喂区域，Feeder 关闭
RTL
LAND
FINISHED
TASK_COMPLETED
```

格间飞行保持 Feeder 关闭。地图 pin、身份或 heartbeat 不一致时，Feeder 会立即关闭并 fail-closed。

## 17. 其他 Feeding 模式

### 17.1 全网格

```json
{
  "mode": "FULL_GRID",
  "rows": [],
  "cols": [],
  "selected_cell_ids": []
}
```

FULL_GRID 要求所有目标 cell 都是 confirmed，否则拒绝任务。

### 17.2 指定一行

```json
{
  "mode": "ROW_OR_COLUMN",
  "rows": [3],
  "cols": [],
  "selected_cell_ids": []
}
```

### 17.3 指定一列

```json
{
  "mode": "ROW_OR_COLUMN",
  "rows": [],
  "cols": [5],
  "selected_cell_ids": []
}
```

### 17.4 指定多个 cell

```json
{
  "mode": "SELECTED_CELLS",
  "rows": [],
  "cols": [],
  "selected_cell_ids": ["R01C01", "R02C04", "R06C10"]
}
```

## 18. 安全取消 Feeding

```bash
curl -s -X POST \
  'http://127.0.0.1:8081/api/cancel' \
  -H 'Content-Type: application/json' \
  -d '{}' \
  | python3 -m json.tool
```

系统会关闭 Feeder、释放 map lease，并由 Mission FSM 执行安全恢复。

## 19. 查看系统综合状态

```bash
curl -s \
  'http://127.0.0.1:8081/api/status' \
  | python3 -m json.tool
```

常用 ROS 检查命令：

```bash
rostopic echo -n 1 /scout/task_status
rostopic echo -n 1 /scout/active_map_status
rostopic echo -n 1 /feeding/task_status
rostopic echo -n 1 /mission/state
rostopic echo -n 1 /feeder/status
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
rostopic echo -n 1 /mavros/local_position/pose
```

列出地图相关服务：

```bash
rosservice list | grep '^/scout/' | sort
```

核心服务：

```text
/scout/list_maps
/scout/activate_map
/scout/manage_map_lease
/scout/reset_map
/scout/finalize_map
```

## 20. Scout 产物保存位置

每个 Scout 任务的输出目录：

```text
/home/ubuntu/.ros/cage_uav/scout_runs/<task_id>/
```

查看所有任务：

```bash
ls -lah /home/ubuntu/.ros/cage_uav/scout_runs
```

一个完整任务通常包括：

```text
request.json
route.json
map.json
quality.json
result.json
manifest.json
events.jsonl
mission.bag
```

不要直接编辑已经完成任务的文件。修改任意字节都会导致 manifest SHA-256 校验失败。

当前 active selector：

```text
/home/ubuntu/.ros/cage_uav/scout_runs/.active_map.json
```

查看 selector：

```bash
python3 -m json.tool \
  /home/ubuntu/.ros/cage_uav/scout_runs/.active_map.json
```

不要手工修改 selector。地图切换必须通过 Web 或 `/scout/activate_map` 服务完成。

## 21. 验证重启恢复

1. 先记下当前 active：

```bash
rostopic echo -n 1 /scout/active_map_status
```

2. 在运行 `roslaunch` 的终端按 `Ctrl+C`。

3. 等待所有节点退出后，再执行第 5 节的完整启动命令。

4. 检查：

```bash
rostopic echo -n 1 /scout/active_map_status
```

正常时 message 包含：

```text
Scout map restored from selector
```

并且重启前后的 UUID、version、map hash 和 manifest hash 完全一致。

## 22. 正确停止系统

在运行 `roslaunch` 的终端按：

```text
Ctrl+C
```

等待所有节点退出。不要在 Scout、Feeding、RTL 或 LAND 过程中直接关闭终端或强制杀死 Gazebo。

如果只是执行了 `rostopic echo` 或 `rostopic hz`，按 `Ctrl+C` 只会停止该监控命令，不会停止仿真。

## 23. 常见错误处理

### 23.1 找不到 launch 文件

错误：

```text
RLException: [persistent_lattice_sim.launch] is neither a launch file...
```

原因通常是没有 source 正确工作区。执行：

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
source /home/ubuntu/cage_uav_ws/devel/setup.bash
rospack find cage_simulation
```

预期：

```text
/home/ubuntu/cage_uav_ws/src/cage_simulation
```

### 23.2 Python setuptools/importlib_metadata 编译错误

执行：

```bash
export PYTHONNOUSERSITE=1
source /opt/ros/noetic/setup.bash
catkin_make -j4
```

### 23.3 Web 打不开

检查节点和端口：

```bash
rosnode list | grep cage_web_bridge
ss -ltnp | grep -E ':808[1-9]|:809[0-9]'
curl -v --max-time 5 http://127.0.0.1:8081/api/status
```

### 23.4 `NO_ACTIVE_MAP` 或 `SCOUT_MAP_NOT_READY`

先列出地图并激活一张 compatible 地图：

```bash
curl -s 'http://127.0.0.1:8081/api/scout/maps?cage_id=0&include_invalid=true' | python3 -m json.tool
```

### 23.5 `REGISTRY_HEARTBEAT_STALE`

检查 registry：

```bash
rosnode ping -c 1 /scout_map_registry
rostopic hz /scout/active_map_status
```

如果 registry 不存在，安全停止并重新启动完整 launch。不要绕过 heartbeat 检查。

### 23.6 `MAP_HASH_MISMATCH`

重新执行地图列表命令，从列表复制最新的 UUID、version、map SHA-256 和 manifest SHA-256。不要手工猜测或只使用 version。

### 23.7 `MAP_LEASE_CONFLICT` 或 `MAP_SWITCH_BUSY`

当前有 Feeding 任务持有地图 lease，不能切换 active 地图。先等待任务完成，或通过 `/api/cancel` 安全取消 Feeding。

### 23.8 `MISSION_BUSY`、`SCOUT_BUSY` 或 `FEEDING_BUSY`

查看：

```bash
rostopic echo -n 1 /mission/state
rostopic echo -n 1 /scout/task_status
rostopic echo -n 1 /feeding/task_status
```

等待当前任务进入终态后再发起下一任务。

### 23.9 task ID 已存在

Scout 的 `task_id` 不能重复，因为任务目录是不可覆盖的审计产物。换一个新 ID，例如：

```text
scout-20261010-001
```

### 23.10 Gazebo 已经启动，又误启动第二套

检查：

```bash
pgrep -af 'roslaunch cage_simulation persistent_lattice_sim.launch'
pgrep -af gzserver
```

回到原来的 roslaunch 终端继续使用，不要重复启动。

## 24. 当前验收结果

当前实现已经验证：

- 历史 v1 地图严格校验和激活；
- active 地图等待超过 60 秒仍可 Preview/Start；
- 两格 Feeding 完整执行；
- 格间 Feeder 自动关闭；
- Feeding map lease acquire/renew/release；
- 地图 pin 全链路传递；
- 错 UUID/hash 拒绝且旧 active 保持；
- registry heartbeat 丢失后 fail-closed；
- 新 Scout 生成 schema v2 artifact；
- Scout 自动 RTL、降落和解除武装；
- 新地图通过同一严格服务自动激活；
- 完整重启后 selector 恢复；
- Gazebo 正常运行；
- `/mavros/local_position/pose` 约 20 Hz。

更详细的设计和验收记录见：

```text
/home/ubuntu/cage_uav_ws/SCOUT_MAP_RELOAD_FEEDING_PERSISTENCE_PLAN.md
/home/ubuntu/cage_uav_ws/SCOUT_MAP_RELOAD_FEEDING_PERSISTENCE_VALIDATION.md
```
