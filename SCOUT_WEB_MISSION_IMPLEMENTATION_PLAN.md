# Scout 网页任务、持久格网输出与自动返航降落实施计划

> 状态：Codex 实施规范（后续编码以本文为准）  
> 日期：2026-10-09  
> 工程：`/Users/yudapao/cage_uav_ws_remote`  
> 目标主机：`ubuntu@90-3:/home/ubuntu/cage_uav_ws`  
> 方法依据：`Scout_Feeder_视觉方法设计.pdf`  
> 上位设计：`SCOUT_FEEDER_IRREGULAR_LATTICE_SIMULATION_PLAN.md`

## 0. 一句话目标

用户在网页点击“开始 Scout 探测 CAGE_0”后，系统只创建一个 Scout 任务；无人机沿侦察航线经过网箱，依靠连续位姿和局部视觉观测建立持久 row-column 格网；航线结束后冻结并保存地图，自动返回任务起飞点、降落、上锁；只有“航线完成 + 地图合格并落盘 + 返航 + 落地 + 上锁”全部成立，网页才显示 `COMPLETED`。

## 1. 本文解决什么

### 1.1 用户明确要求

1. Scout 是网页中的独立按钮，不是启动 launch 后自动执行。
2. 用户指定目标网箱后才能开始探测。
3. Scout 飞行结束后必须自动返回起飞点。
4. 返回后必须自动降落并上锁。
5. 网页必须显示 Scout 进度、地图结果、返航、降落和上锁状态。
6. Scout 的最终输出必须符合 PDF 的持久格网方法，而不是只输出视频、检测框或一条航迹。

### 1.2 PDF 与用户新增要求的边界

PDF 定义的是 Scout/Feeder 的视觉身份方法和地图输出；网页按钮、任务状态机、RTL、降落和上锁是本项目新增的飞行任务编排要求。两部分都必须实现，但不能把后者说成 PDF 原文已有内容。

### 1.3 第一版目标网箱范围

第一版只允许探测 `CAGE_0`，原因不是网页限制，而是当前后端事实：

- `multi_cage_layout.yaml` 只有 `CAGE_0` 为 `targetable: true`；
- ground-truth publisher、oracle observation、Scout map 和当前 route commander 都固定为 `cage_id=0`；
- 其余四个网箱尚未建立独立 namespace 或动态目标切换。

因此网页先诚实显示一个目标：`CAGE_0`。不能只增加五个按钮却仍让后端全部探测 cage 0。多网箱动态选择作为后续扩展，必须先完成节点命名空间或动态 session 隔离。

## 2. 当前已经具备的能力

以下内容已经实现并在 90-3 Gazebo 在线验证，不应重写：

- 不规则共享顶点网箱及 mild/medium/strong 形变；
- `/sim_gt/cage/lattice` 与算法 topic 隔离；
- `oracle_noisy` 局部观测；
- Top-K 整数偏移、持久 cell 身份、可见性状态、边界 birth 和漏检恢复；
- 坐标优化、协方差、版本化 `PersistentLattice`；
- Feeder 候选、拒判和投料门禁；
- `mock_mavros_node.py` 持续发布 `/mavros/local_position/pose`、GPS、速度，并同步 Gazebo 模型；
- 12 个蛇形侦察航点已经实际飞完；
- 在线结果已达到 60/60 confirmed、0 missing、估计位置 RMSE 约 0.0392 m。

这些是 Scout 地图后端和运动仿真的基线。本轮新增的是“网页驱动的完整任务生命周期”和“不可变最终任务结果”。

## 3. 当前差距与根因

### 3.1 Scout 目前不是网页任务

`src/cage_simulation/scripts/scout_route_commander.py` 当前行为是：

- launch 后拿到 GT 和 GPS 就自动执行；
- 直接调用 MAVROS clear/push/set_current/arm/set_mode；
- 只用一个 `started` 布尔值防止重入；
- 不订阅 `/mavros/mission/reached`；
- 不知道最后一个航点何时完成；
- 不发布 Scout task status/result；
- 不请求 RTL；
- 无法开始第二个干净的新任务。

`persistent_lattice_sim.launch` 又把它配置为 `auto_start=true`，因此与“网页按钮触发”直接冲突。

### 3.2 网页现有 Start 只表示投喂

当前 Web 只有：

```text
POST /api/preview
POST /api/start
POST /api/cancel
```

它们发布 `/feeding/*`，不是 Scout。网页虽然显示 `/scout/lattice_map` 和 offset 摘要，但没有 Scout request、cancel、mission status 或 result。

### 3.3 当前飞行控制存在双写风险

完整 system 已启动 `mission_fsm_node.py`，但 Scout commander 又直接写 MAVROS。这样会出现：

- 无人机已经在 AUTO 飞行，网页 FSM 仍显示 `IDLE`；
- Scout 与投喂任务可能同时覆盖 FCU mission；
- RTL、manual takeover、timeout recovery 无法统一；
- Web 或两个节点可能同时改变飞控模式。

本轮必须消除这个架构问题，不能只在旧 commander 末尾补一行 `set_mode('RTL')`。

### 3.4 地图没有任务级 reset/finalize

当前 `scout_lattice_node.py` 可以持续发布地图，并可配置为逐帧覆盖一个 JSON，但没有：

- 新任务创建新 map UUID 的 reset；
- 指定 task/mission 的原子 finalize；
- 不可变最终地图版本；
- 本次任务独立的 result/manifest；
- 防止把上一次的 60/60 当成本次飞行结果的 session 边界。

## 4. 强制架构：只能有一个飞控写入者

采用以下数据流：

```text
Web Scout button
       |
       v
/scout/task_request
       |
       v
ScoutTaskManager + Scout route planner
       |  发布通用 MissionRequest / MissionWaypointArray
       v
mission_fsm_node.py
       |
       v
MavrosAdapter
       |
       v
mock MAVROS / real MAVROS / FCU
```

硬规则：

1. Web 只发布高层 Scout 请求，绝不直接调用 MAVROS。
2. ScoutTaskManager 负责任务编排、路线和产物，不直接 arm、set mode 或写 mission service。
3. `mission_fsm_node.py` 是自动飞行期间唯一的 MAVROS 控制写入者。
4. `scout_route_commander.py` 必须移除直接 MAVROS `ServiceProxy`；可改造成 ScoutTaskManager/route publisher，或由新文件替代后删除 launch 中的旧执行路径。
5. manual takeover 永远优先；进入 `MANUAL_OVERRIDE` 后任何 Scout 节点都不得自动抢回 AUTO。

## 5. 端到端流程

### 5.1 启动后

- Gazebo、mock MAVROS、持久格网节点、任务 FSM、Web 和 ScoutTaskManager 常驻。
- 无人机保持 home、地面和未上锁。
- Scout 状态为 `IDLE`。
- 等待 10 秒也不能自动起飞。

### 5.2 网页点击开始

网页调用：

```http
POST /api/scout/start
Content-Type: application/json

{
  "task_id": "scout-web-20261009-001",
  "cage_id": 0,
  "survey_relative_alt": 25.0,
  "auto_rtl": true,
  "auto_land": true
}
```

服务端返回 HTTP `202`：

```json
{
  "accepted": true,
  "task_id": "scout-web-20261009-001",
  "state": "PRECHECK"
}
```

以下情况拒绝：

- 非法 JSON 或字段：`400`；
- cage 不可执行：`400` 或 `409`；
- Scout/Feeder/其他 mission 正在运行：`409`；
- 相同 task ID 重放：返回原任务状态，或明确 `409`，不能创建第二次任务；
- `auto_rtl=false` 或 `auto_land=false`：第一版直接 `400`，不允许网页关闭安全收尾。

### 5.3 后端执行

1. 预检 FCU、GPS、本地位姿、控制权、观察链 heartbeat、输出目录和任务互斥。
2. 记录任务开始时的 home GPS 与 local pose。
3. reset Scout map，创建本次 task 的新 map UUID。
4. 从目标网箱的名义 survey region 生成蛇形覆盖航线。
5. 通过通用 MissionRequest/MissionWaypointArray 交给 mission FSM。
6. mission FSM 上传航线、解锁、起飞并进入 AUTO。
7. UAV 沿路线飞行；mock 持续发布真实变化的 pose，oracle/vision 根据 pose 产生局部观测。
8. 最后一个 Scout 航点确认到达后，冻结地图并写任务产物。
9. 无论地图成功、地图失败还是用户在空中取消，都进入安全 RTL。
10. 返回任务开始记录的 home 附近后进入/确认 LAND。
11. 检查落地并上锁。
12. 写完 result 和 manifest 后发布最终状态。

## 6. 两层状态机

飞行安全继续由通用 mission FSM 负责；ScoutTaskManager 负责业务结果。两者通过相同 `mission_id` 关联。

### 6.1 正常状态

```text
IDLE
  -> PRECHECK
  -> PLANNING
  -> UPLOADING
  -> ARMING
  -> TAKEOFF
  -> TRANSIT
  -> SCOUTING
  -> FINALIZING_MAP
  -> RTL_RETURNING
  -> LANDING
  -> DISARMED
  -> COMPLETED
```

### 6.2 失败或取消

```text
起飞前失败
  -> FAILED

起飞后任意自动状态失败/取消
  -> ABORTING
  -> RTL_RETURNING
  -> LANDING
  -> DISARMED
  -> FAILED 或 CANCELED
```

地图质量失败也必须返航落地：

```text
SCOUTING -> FINALIZING_MAP -> FAILED_MAP_QUALITY_PENDING_RECOVERY
          -> RTL_RETURNING -> LANDING -> DISARMED -> FAILED
```

### 6.3 手动接管

```text
任意自动状态 -> MANUAL_OVERRIDE
```

进入后：

- 停止自动 mission/mode 请求；
- 不抢回 AUTO；
- 网页明确显示“人工控制中”；
- 只有人工通过既有 control-authority 流程明确交还控制后，才能开始新的任务；
- 第一版不自动续飞旧 Scout 任务。

## 7. ROS 数据合同

### 7.1 新增 `ScoutTaskRequest.msg`

建议路径：`src/cage_msgs/msg/ScoutTaskRequest.msg`

```text
std_msgs/Header header
string task_id
int32 cage_id
float64 survey_relative_alt
bool auto_rtl
bool auto_land
```

Topic：

```text
/scout/task_request
```

取消使用独立 topic：

```text
/scout/task_cancel    std_msgs/String   # data=task_id
```

取消的语义是“安全中止并返航”，不是让无人机立即停在空中。

### 7.2 新增 `ScoutTaskStatus.msg`

建议路径：`src/cage_msgs/msg/ScoutTaskStatus.msg`

至少包含：

```text
std_msgs/Header header
string task_id
string mission_id
int32 cage_id
string state
string reason
string error_code

uint32 current_waypoint
uint32 total_waypoints
bool route_completed

string map_uuid
uint64 map_version
string geometry_sha256
uint32 confirmed_cells
uint32 expected_cells
float32 coverage
bool map_finalized
bool map_ready_for_feeding

bool rtl_commanded
bool rtl_accepted
bool returned_home
float32 home_horizontal_error_m
bool landed
bool disarmed

string output_directory
```

Topic：

```text
/scout/task_status    # latched
```

浏览器只显示后端状态，不能自行根据计时器推断“已完成”。

### 7.3 扩展通用 mission 消息

在 `MissionRequest.msg` 和 `MissionState.msg` 增加任务类型。推荐使用常量：

```text
uint8 MISSION_FEEDER=0
uint8 MISSION_SCOUT=1
uint8 mission_type
bool auto_land
```

兼容要求：旧调用未赋值时默认 `MISSION_FEEDER`，现有 rectangular/circular feeding 行为不变。

Scout waypoint 链使用：

```text
TAKEOFF
TRANSIT
SCOUT_ENTER
SCOUT_PASS      # 1..N
SCOUT_EXIT
```

`SCOUT_PASS` 永远不能打开 feeder。

### 7.4 地图 reset/finalize service

新增两个明确接口，不能靠删除文件或重启节点模拟 session：

```text
/scout/reset_map
/scout/finalize_map
```

建议新增：

```text
src/cage_msgs/srv/ResetScoutMap.srv
src/cage_msgs/srv/FinalizeScoutMap.srv
```

Reset 请求至少含 `task_id/cage_id`，响应返回新 `map_uuid`。Finalize 请求至少含 `task_id/cage_id/output_directory/min_coverage`，响应返回：

```text
success
reason
map_uuid
map_version
confirmed_cells
expected_cells
coverage
snapshot_path
```

Finalize 必须在锁内取得一个确定版本的深拷贝，然后以临时文件 + `fsync` + 原子 rename 写盘。返回成功后的 snapshot 不得再修改。

### 7.5 继续保留的算法 topic

```text
/scout/local_lattice_observation
/scout/offset_hypotheses
/scout/lattice_map
/scout/map_events
/scout/route_path
/mavros/local_position/pose
/mavros/global_position/global
/mavros/mission/reached
/mavros/state
/mavros/extended_state
```

## 8. 路线生成约束

### 8.1 路线来源

默认路线从已知的名义 survey region 生成，例如 `/cage_grid` 的 anchor、rows、cols、cell size 和 yaw。路线只是保证相机覆盖目标区域，不需要知道实际 cell 身份。

禁止：

- 用已完成的 `/scout/lattice_map` 反过来生成首次 Scout 路线；
- 把 `/sim_gt` 的 `(row,col)` 或真实坐标注入 Scout 关联算法；
- 在真实飞行节点中依赖 `/sim_gt/cage/lattice`。

仿真 evaluator 可以离线读取 GT；如果测试工具临时用 GT 边界生成覆盖路线，必须标记 `route_source=sim_gt_bounds`，并且 GT 仍不得进入 Scout identity/map 更新链。正式默认应为 `route_source=nominal_survey_region`。

### 8.2 CAGE_0 第一版路线

- 6 行，每行两个端点，蛇形交替，共 12 个 survey pass endpoint；
- 默认相对高度 25 m；
- 路线经过网箱上方，不绕过网箱；
- 相机 footprint 与 pass spacing 必须保证覆盖相邻行；
- 继续发布 `/scout/route_path` 供 RViz/Web 调试；
- 航线终点不是任务终点，只有后续 map finalize、RTL、land、disarm 成功才结束。

## 9. Scout 最终输出

### 9.1 方法核心输出

Scout 最终交给 Feeder 的不是检测框，而是持久 row-column lattice：

- 永久稳定的 `(row,col)` 和 `cell_id`；
- nominal position；
- 修正后的 ENU estimated position；
- `covariance_xy`；
- `tentative/confirmed/rejected` 身份状态；
- `visible/out_of_view/expected_but_unobserved` 可见性状态；
- row/column 邻接边；
- 局部格网到持久格网的 Top-K 整数偏移、后验和提交/拒绝原因；
- `map_uuid/map_version/anchor_definition/geometry_sha256`。

身份状态和可见性状态必须分开。暂时看不见 confirmed cell 不能删除其永久身份。

### 9.2 每次任务的目录

默认根目录建议为：

```text
$(env HOME)/.ros/cage_uav/scout_runs/<task_id>/
```

每个任务至少生成：

```text
request.json
route.json
map.json
events.jsonl
quality.json
result.json
manifest.json
mission.bag          # 可配置关闭，但集成验收应开启
```

写入顺序：

```text
map -> quality -> result -> manifest
```

全部关键 JSON 使用临时文件、flush、fsync 和原子 rename。只有 manifest 成功写入后才能发布 `COMPLETED`。失败或取消也必须保留 request、events、已有 bag 和 result。

### 9.3 `result.json` 合同

```json
{
  "schema_version": 1,
  "task_id": "scout-web-20261009-001",
  "mission_id": "scout-web-20261009-001-flight",
  "cage_id": 0,
  "terminal_status": "COMPLETED",
  "reason": "",
  "flight_outcome": "SUCCESS",
  "mapping_outcome": "SUCCESS",
  "flight": {
    "waypoints_total": 12,
    "waypoints_reached": 12,
    "route_completed": true,
    "rtl_commanded": true,
    "rtl_accepted": true,
    "returned_home": true,
    "home_horizontal_error_m": 0.18,
    "landed": true,
    "disarmed": true
  },
  "map": {
    "map_uuid": "...",
    "map_version": 500,
    "frozen": true,
    "geometry_sha256": "...",
    "expected_cells": 60,
    "confirmed_cells": 60,
    "tentative_cells": 0,
    "rejected_cells": 0,
    "coverage": 1.0
  },
  "quality": {
    "ambiguous_frame_count": 0,
    "mean_identity_confidence": 0.98,
    "optimizer_rmse_m": 0.04,
    "optimizer_outlier_count": 0
  },
  "artifacts": {
    "persistent_map": "map.json",
    "events": "events.jsonl",
    "rosbag": "mission.bag",
    "quality_report": "quality.json",
    "manifest": "manifest.json"
  }
}
```

必须分别报告：

- `flight_outcome`：航线、返航、落地是否成功；
- `mapping_outcome`：地图是否达到门槛并成功冻结；
- `terminal_status`：两者都成功才是 `COMPLETED`。

如果地图不合格但飞机安全返航，应显示 `FAILED_MAP_QUALITY`，同时保留 `landed=true/disarmed=true`，不能伪装成功。

### 9.4 `map.json` 合同

顶层至少保存：

```text
schema_version, task_id, cage_id, map_uuid, map_version, frozen,
frame_id, anchor_definition, anchor_enu, geometry_sha256,
configuration_sha256, observation_source, created_at, finalized_at,
cells[], edges[]
```

每个 cell 至少保存：

```text
row, col, cell_id,
identity_state, visibility_state,
nominal_position, estimated_position, position_delta,
covariance_xy, polygon,
identity_confidence,
first_seen, last_seen,
observation_count, confirmation_count
```

生产地图不能保存 GT identity 或 GT coordinate。GT 只允许 evaluator 读取。

### 9.5 `manifest.json`

保存：

- 所有产物的 SHA-256；
- preset、seed、配置哈希；
- map UUID/version；
- observation source；
- 源码 Git commit；本地无 Git 元数据时记录源码树 hash；
- 开始、航线完成、地图冻结、返航、落地和结束时间。

## 10. 完成门槛

### 10.1 在线任务门槛

只有同时满足以下条件才能发布 `COMPLETED`：

1. 所有 Scout 航点都收到对应的 reached 确认。
2. 最终地图取得确定版本、通过质量门并原子落盘。
3. RTL 请求已发送且飞控确认接受。
4. 飞机返回任务开始前记录的 home 附近。
5. 真实飞行优先由 `/mavros/extended_state` 确认 `ON_GROUND`。
6. 相对高度低于阈值。
7. `/mavros/state.armed == false`，且连续稳定若干采样周期。
8. result 和 manifest 成功落盘。

以下任何单项都不代表完成：

- 到达最后一个航点；
- `set_mode(RTL)` 返回成功；
- mode 字符串等于 RTL；
- 相对高度接近零；
- 地图曾经出现 60/60；
- Gazebo 画面看起来像已落地。

### 10.2 CAGE_0 仿真默认阈值

```yaml
scout_mission:
  targetable_cages: [0]
  survey_relative_alt_m: 25.0
  expected_cells: 60
  minimum_coverage: 0.95
  finalize_settle_sec: 1.0
  finalize_timeout_sec: 10.0
  rtl_timeout_sec: 180.0
  land_timeout_sec: 120.0
  home_radius_m: 1.0
  landed_relative_alt_m: 0.5
  disarm_stable_sec: 1.0
```

完整 6 x 10 oracle-noisy 验收仍要求 60/60；`minimum_coverage=0.95` 是运行期是否保留可诊断结果的最低门，不是降低集成验收标准。

### 10.3 PDF 方法评测指标

离线仿真/标注评测至少报告：

1. row/column identity accuracy；
2. identity switch 和整行/整列 `+-1` slip rate；
3. new-cell birth precision/recall；
4. missed-cell recovery rate；
5. coordinate refinement RMSE，以及相对 nominal 的 improvement。

真实飞行没有 GT 时，只能报告 coverage、confidence、covariance、ambiguity count 和 optimizer residual 等在线可观测量。禁止伪造真实飞行 identity accuracy 或 GT RMSE。

## 11. 自动返航与降落细节

### 11.1 Home

- task 进入 PRECHECK 时保存当前 GPS 和 local pose；
- FCU home 仍由飞控负责，保存值用于验收和状态显示；
- `returned_home` 必须用水平距离判断，不能用相对高度替代。

### 11.2 RTL

- 正常航线结束、空中取消和空中自动错误都请求 RTL；
- 请求要有限重试并验证 mode；
- RTL 超时进入失败恢复状态并保留原因；
- 手动接管时不得发送 RTL，除非现有人工控制流程明确请求。

### 11.3 LAND 与 DISARM

- 如果 RTL 配置会自动落地，则持续观察 landed/disarmed；
- 如果回到 home radius 后仍保持高度，自动请求 `LAND`；
- `MavrosAdapter` 增加 `land(mode='LAND')`；
- `mock_mavros_node.py` 增加明确 LAND 行为和 `/mavros/extended_state`；
- mock RTL 建议先返回 home，再下降，更接近真实流程；
- 落地后确认上锁，不允许只靠一次采样；
- 非预期在远离 home 处 disarm 应记录为 flight failure，而不是成功。

## 12. Web 设计

### 12.1 HTTP API

新增并保留现有 feeding API：

```text
POST /api/scout/start
POST /api/scout/cancel
GET  /api/status            # 增加 scout_task 字段
```

不复用 `/api/start`，防止用户和代码混淆投喂与侦察。

`/api/status` 增加：

```json
{
  "scout_task": {
    "can_start": true,
    "can_cancel": false,
    "task_id": "",
    "cage_id": 0,
    "state": "IDLE",
    "waypoints": {"reached": 0, "total": 12},
    "map": {"confirmed": 0, "expected": 60, "coverage": 0.0},
    "route_completed": false,
    "map_finalized": false,
    "rtl_accepted": false,
    "returned_home": false,
    "landed": false,
    "disarmed": true,
    "reason": "",
    "error_code": ""
  }
}
```

### 12.2 页面

增加独立 Scout 卡片，不改名或复用 Feeder 的 Start Task：

- 目标网箱：第一版固定/下拉只有 `CAGE_0`；
- 侦察高度：默认 25 m；
- “开始 Scout 探测”按钮；
- “安全取消并返航”按钮；
- 航点 `n/12`；
- confirmed/expected 与 coverage；
- map UUID/version/age；
- offset Top-1 与 confidence；
- route/map/RTL/home/land/disarm 六个状态；
- reason/error 和 output directory。

运行中禁用开始按钮。只有 `IDLE/COMPLETED/FAILED/CANCELED` 可开始新任务。前端禁用只是 UX，后端必须再次做互斥检查。

## 13. 按文件实施

### 13.1 消息与服务

新增：

```text
src/cage_msgs/msg/ScoutTaskRequest.msg
src/cage_msgs/msg/ScoutTaskStatus.msg
src/cage_msgs/srv/ResetScoutMap.srv
src/cage_msgs/srv/FinalizeScoutMap.srv
```

修改：

```text
src/cage_msgs/msg/MissionRequest.msg
src/cage_msgs/msg/MissionState.msg
src/cage_msgs/CMakeLists.txt
src/cage_msgs/package.xml
```

### 13.2 Scout 编排与路线

修改或替代：

```text
src/cage_simulation/scripts/scout_route_commander.py
```

推荐重命名为：

```text
src/cage_simulation/scripts/scout_task_manager_node.py
```

职责：请求验证、session、路线生成、通用 mission 发布、地图 finalize、产物、Scout status。不得含任何 MAVROS arm/mode/mission `ServiceProxy`。

将纯路线算法放入可单测模块，例如：

```text
src/cage_waypoint_planner/src/cage_waypoint_planner/scout_planner.py
```

`waypoint_planner_node.py` 必须按 `mission_type` 路由，确保 Scout 和 Feeder 不会同时发布不同航线。

### 13.3 通用 mission FSM

修改：

```text
src/cage_mission_fsm/src/cage_mission_fsm/core.py
src/cage_mission_fsm/scripts/mission_fsm_node.py
src/cage_mavros_adapter/src/cage_mavros_adapter/adapter.py
```

增加：

- `SCOUTING`、`FINALIZING_MAP`；
- Scout waypoint 校验；
- map finalize handshake；
- home 水平距离；
- RTL 确认；
- 必要时 LAND 请求；
- landed + disarmed 稳定确认；
- failure/cancel 安全恢复；
- Scout 全程 feeder 强制关闭。

### 13.4 地图与记录

修改：

```text
src/cage_grid/scripts/scout_lattice_node.py
src/cage_grid/src/cage_grid/persistent_map.py
src/cage_bringup/scripts/persistent_lattice_event_logger.py
src/cage_bringup/scripts/evaluate_persistent_lattice.py
```

增加 reset/finalize、任务边界、不可变快照、`RECOVER` 事件，并统一 logger 的 `record_type` 与 evaluator 的输入 schema。目前两者一个用 `record_type`、一个用 `type`，必须消除不兼容。

### 13.5 Web

修改：

```text
src/cage_web_bridge/scripts/web_bridge_node.py
src/cage_web_bridge/www/index.html
src/cage_web_bridge/config/web.yaml
```

增加 Scout pub/sub、HTTP endpoints、服务端门禁和页面状态。

### 13.6 仿真与 launch

修改：

```text
src/cage_simulation/scripts/mock_mavros_node.py
src/cage_simulation/launch/persistent_lattice_sim.launch
src/cage_simulation/config/persistent_lattice_sim.yaml
src/cage_simulation/CMakeLists.txt
```

要求：

- Scout manager 常驻但默认不自动开始；
- 旧 `start_scout_route` 如保留，只表示是否启动 manager，不表示立即起飞；
- 新增 `scout_auto_start:=false`，只允许 CI 显式设 true；
- launch 记录 Scout request/status/result、extended state；
- `start_web` 参数必须真正控制 Web，而不是名义上传参后仍总是启动。

## 14. Codex 必须按此顺序实施

### 阶段 A：冻结基线

1. 先检查目标主机当前源码，不假设本地副本一定最新。
2. 保存远端源码快照/差异，不能覆盖用户已有修改。
3. 运行现有纯 Python tests 和一次 current persistent-lattice acceptance。
4. 记录当前 launch、topic、60/60 和 RMSE 基线。

### 阶段 B：消息与纯状态机

1. 新增 Scout request/status 和 map service。
2. 扩展 MissionRequest/State 的 mission type。
3. 先写不依赖 ROS transport 的 Scout/FSM 单元测试。
4. 证明旧 feeding message 的默认值仍走 FEEDER。

### 阶段 C：单控制器重构

1. 从旧 commander 删除 direct MAVROS 写入。
2. Scout manager 发布通用 mission，mission FSM 单独控制 FCU。
3. 加入任务互斥和 task ID 幂等。
4. 默认启动后等待网页，不自动飞。

### 阶段 D：地图 session 与产物

1. 新任务 reset 新 UUID。
2. 最后航点进入 FINALIZING_MAP。
3. 原子冻结 map/version。
4. 写 quality/result/manifest 和校验和。
5. finalize 失败仍进入 RTL，但最终任务失败。

### 阶段 E：RTL、LAND、DISARM

1. 记录实际 task home。
2. 航线完成后请求并确认 RTL。
3. 回 home 后确认或请求 LAND。
4. 用 extended state、相对高度、armed 联合判断。
5. 补 mock RTL/LAND/extended-state。

### 阶段 F：Web

1. 新增独立 endpoints。
2. 新增 Scout 卡片和状态。
3. 后端处理 400/409/202 和幂等。
4. 保持现有 Feeder UI/API 回归兼容。

### 阶段 G：90-3 集成验收

1. 同步到 `/home/ubuntu/cage_uav_ws` 前先备份。
2. `catkin_make`。
3. 在 90-3 启动 Gazebo、Web 和完整系统。
4. 用 Web 和 curl 各执行一次任务。
5. 保留日志、bag、result 和验收数据。

## 15. 测试与硬验收

### 15.1 必加测试

```text
test_scout_task_fsm.py
test_scout_route_planner.py
test_scout_map_finalize.py
test_scout_web_api.py
mock RTL/LAND rostest
HTTP -> ROS -> Gazebo 端到端测试
```

覆盖：

- 启动后不自动飞；
- 单击只创建一个 mission；
- 运行中重复开始返回 409；
- 非 cage 0 请求被拒绝；
- route 12/12；
- map 60/60；
- map reset 后 UUID 改变；
- finalize snapshot 可重新加载且 UUID/version/cell count 一致；
- finalize 写盘失败仍返航、落地、上锁，结果为 FAILED；
- 空中 cancel 安全 RTL；
- 手动 takeover 不抢权；
- RTL 超时和 LAND 超时有明确 reason；
- Feeder 在 SCOUTING 中始终关闭；
- 旧 rectangular/circular feeding 回归通过。

### 15.2 90-3 最终场景

启动命令在实施完成后应类似：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ubuntu/cage_uav_ws
catkin_make
source devel/setup.bash

roslaunch cage_simulation persistent_lattice_sim.launch \
  deformation_preset:=medium \
  deformation_seed:=42 \
  observation_source:=oracle_noisy \
  feeder_visual_source:=oracle_noisy \
  scout_auto_start:=false \
  start_rviz:=true \
  start_web:=true
```

然后调用实际 Web 监听端口：

```bash
curl -sS -X POST http://127.0.0.1:8081/api/scout/start \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id":"scout-acceptance-001",
    "cage_id":0,
    "survey_relative_alt":25.0,
    "auto_rtl":true,
    "auto_land":true
  }'
```

注意：8081 只是当前 90-3 已知可用端口。实现后应在启动日志中读取实际端口，或让 launch 明确固定端口，不能盲目假定。

验收必须观察到完整顺序：

```text
IDLE -> PRECHECK -> ... -> SCOUTING -> FINALIZING_MAP
-> RTL_RETURNING -> LANDING -> DISARMED -> COMPLETED
```

最终数值：

```text
waypoints_reached = 12/12
confirmed_cells = 60/60
route_completed = true
map_finalized = true
rtl_accepted = true
home_horizontal_error_m <= 1.0
relative_altitude_m <= 0.5
landed = true
armed = false
Gazebo/ROS pose error <= 0.05 m
/mavros/local_position/pose >= 10 Hz during the full mission
result.json and manifest.json exist and hashes verify
```

### 15.3 不允许的“假通过”

- 手动在终端发送 RTL 后声称网页流程完成；
- 看到 map 曾为 60/60 就复用旧地图；
- 直接 teleport UAV 回原点；
- 停止 mock pose publisher；
- 用 GT 直接填 Scout identity/estimated position；
- 只测试 curl，不测试网页按钮；
- 只测试 Web 状态，不观察 Gazebo/ROS 实际运动；
- 未落地或仍 armed 就标记 `COMPLETED`；
- 修改测试门槛来掩盖失败。

## 16. Definition of Done

只有以下全部成立，本功能才完成：

- 默认 launch 不再自动执行 Scout；
- 网页有独立“开始 Scout 探测”按钮；
- 后端只接受 targetable `CAGE_0`；
- Scout 和 Feeder 任务互斥；
- 自动飞行只有 mission FSM/MavrosAdapter 一个写入者；
- UAV 真实沿侦察航线经过网箱；
- `/mavros/local_position/pose` 全程持续发布并驱动观测；
- 每次任务有新 map session，不复用旧 60/60；
- 最终输出稳定 row-column identity、修正坐标、协方差、邻接和多假设决策；
- 最终地图不可变、可重新加载、带 manifest 和 hash；
- 正常完成后自动 RTL、返回 home、LAND、disarm；
- 地图失败或用户取消后仍能安全 RTL/land，但任务结果如实失败/取消；
- 手动接管优先且不会被自动抢权；
- Web 显示真实后端进度和最终产物路径；
- 90-3 Gazebo 端到端验收与旧功能回归全部通过；
- Codex 在报告中给出执行命令、测试结果、关键 topic 证据和剩余未验证项，不能用“代码已写”代替运行验证。

## 17. 给后续 Codex 的直接指令

```text
严格按照 SCOUT_WEB_MISSION_IMPLEMENTATION_PLAN.md 实施。

先审计并备份 90-3 当前 /home/ubuntu/cage_uav_ws，再修改；不得假设本地副本是最新真值。
保留现有不规则持久格网算法，不重写已经验收的 M1-M5/M7。
把 Scout 从启动即飞的一次性 commander 改成网页请求驱动的任务。
Web 不得直接控制 MAVROS；ScoutTaskManager 不得直接控制 MAVROS；自动飞行只由 mission_fsm_node + MavrosAdapter 控制。
默认启动后 UAV 必须保持地面、未上锁，直到网页明确开始。
每个 Scout 任务必须 reset 新地图 session，最后冻结不可变地图和任务产物。
最后航点不等于任务完成；必须继续 RTL、返回 home、降落、上锁并写完 manifest。
空中错误、地图失败或用户取消仍要安全返航落地，但最终状态必须如实为 FAILED/CANCELED。
手动接管优先，禁止自动抢回控制权。
先写纯状态机和服务测试，再接 Web，再到 90-3 做 Gazebo 全链路验证。
只有完整状态序列、12/12、60/60、home/land/disarm、pose 频率、Gazebo 同步和产物 hash 都有证据时才宣布完成。
```
