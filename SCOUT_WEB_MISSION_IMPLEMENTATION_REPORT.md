# Scout Web Mission 实施与 90-3 验收报告

日期：2026-10-09  
实施依据：`SCOUT_WEB_MISSION_IMPLEMENTATION_PLAN.md`  
工作区：本机 `/Users/yudapao/cage_uav_ws_remote`；目标机 `ubuntu@90-3:/home/ubuntu/cage_uav_ws`

## 1. 结论

Scout 第一阶段 CAGE_0 闭环已经实现并在 90-3 Gazebo 中通过：

```text
网页/API 请求
-> PRECHECK -> PLANNING -> UPLOADING -> ARMING -> TAKEOFF
-> TRANSIT -> SCOUTING -> FINALIZING_MAP
-> RTL_RETURNING -> LANDING -> COMPLETED
```

最终任务 `scout-final-acceptance-20261009` 的硬指标：

- 航点：12/12；
- confirmed cells：60/60，coverage=1.0；
- 地图已冻结，map version 在任务结束后保持 472；
- RTL commanded/accepted=true；
- Home 水平误差 0.430 m；
- landed=true，armed=false，disarmed=true；
- `/mavros/local_position/pose` 实测 20 Hz；
- Gazebo `iris_demo` 与 MAVROS pose 三维误差 `1.21e-7 m`；
- `mission.bag` 16.0 MB、128 s，包含 2576 条 local pose、2576 条 FCU state、完整 map/status/event；
- result/manifest/map/quality/events/route/request/bag 均已生成，manifest 中 7 个文件 hash 全部验证通过。

## 2. 主要实现

### 2.1 任务合同与单一飞控写入者

- 新增 `ScoutTaskRequest.msg`、`ScoutTaskStatus.msg`；
- 新增 `ResetScoutMap.srv`、`FinalizeScoutMap.srv`；
- `MissionRequest/State` 新增 `MISSION_FEEDER/MISSION_SCOUT` 和 `auto_land`；
- Web 只发布高层 Scout request/cancel；
- ScoutTaskManager 只生成任务、路线、地图 session 和产物，不调用 MAVROS；
- 唯一的飞控写入链为 `mission_fsm_node.py -> MavrosAdapter`；
- 旧 `scout_route_commander.py` 已改为默认关闭的高层 request 兼容触发器，不再导入或调用 MAVROS，也不读取仿真 GT。

### 2.2 路线、地图和任务产物

- 路线由 nominal survey region 生成，不由 GT 或已建地图生成；
- 默认 6 行蛇形路线，12 个 Scout survey endpoints；
- 每次任务 reset 新 map UUID；
- 最后航点只进入 `FINALIZING_MAP`，不会直接判定任务完成；
- finalize 在锁内取得稳定版本并原子写 `map.json`；
- 正常任务要求 60/60 集成验收；运行期最低覆盖门为 0.95；
- 输出 `request.json`、`route.json`、`events.jsonl`、`map.json`、`quality.json`、`result.json`、`manifest.json`，可选 `mission.bag`；
- manifest 记录每个文件 SHA-256；
- 取消任务也会立即冻结部分地图、写完整诊断产物，再安全 RTL/LAND/DISARM。

### 2.3 Web

- `POST /api/scout/start`：严格校验 task ID、CAGE_0、高度、auto_rtl/auto_land；
- `POST /api/scout/cancel`：按 task ID 安全取消；
- 同 task ID 重试幂等，不重复发布 mission；
- 活动任务期间不同 task ID 返回 HTTP 409；
- Scout 与 Feeding 双向互斥；
- 页面显示航点、地图覆盖、map UUID/version、RTL、Home、落地、上锁和输出目录；
- 页面按钮已通过真实浏览器点击验证，不是只做 curl 测试。

## 3. 90-3 验收证据

最终正常任务目录：

```text
/home/ubuntu/.ros/cage_uav/scout_runs/scout-final-acceptance-20261009
```

正常任务关键结果：

```text
terminal_status: COMPLETED
flight_outcome: SUCCESS
mapping_outcome: SUCCESS
waypoints_reached/total: 12/12
confirmed/expected: 60/60
map_uuid: 112bb2cd-993f-452f-9936-94556a13ac74
map_version: 472
home_horizontal_error_m: 0.430127
landed/disarmed: true/true
HASHES_OK: true
```

网页空中取消任务：

```text
task_id: scout-web-1791512409204
terminal_status: CANCELED
error_code: USER_CANCELED
flight_outcome: CANCELED_SAFE_RECOVERY
mapping_outcome: CANCELED_PARTIAL_MAP
partial map: 8/60, frozen=true, version=29
rtl_accepted/returned_home/landed/disarmed: true/true/true/true
home_horizontal_error_m: 0.400298
```

取消后立即启动下一任务得到新 UUID：

```text
old UUID: 3b4d3eab-cd96-434e-a80e-3515eb53ef11
new UUID: 4f2c134a-4e9c-420f-86d8-fc3f5c07e6c3
```

HTTP 合同实测：

```text
CAGE_1                         -> 409 CAGE_NOT_TARGETABLE
auto_land=false                -> 400 AUTO_LAND_REQUIRED
首次启动                       -> 202 accepted
同 task_id 重试                -> 202 idempotent=true，未重复发布
活动期不同 task_id             -> 409 SCOUT_TASK_BUSY
安全取消                       -> 202 accepted
```

## 4. 测试与回归

- 目标机 `catkin_make`：通过；
- 纯 Python/ROS message 单元测试：82 个通过（原有 78 + Web API 4），0 failure；
- Scout planner：12 survey endpoints、无 GT 输入，通过；
- FSM：正常 finalize、失败后 RTL、取消恢复、手动接管等测试通过；
- persistent map：reset、新 UUID、RECOVER、冻结、原子保存、重新加载测试通过；
- Web API catkin nosetest：4/4 通过；
- mild/medium/strong × 5 seeds，共 15 场景 acceptance：通过；
- identity accuracy=1.0、slip=0、estimated RMSE=0.1095 m、nominal improvement=95.63%；
- Feeder candidate recall=1.0、wrong commit=0、forbidden feed trigger=0；
- rectangular/circular planner 和现有 Feeding message 默认 FEEDER 路径回归单元测试通过。

## 5. 正确命令

90-3 上先构建：

```bash
export PYTHONNOUSERSITE=1
source /opt/ros/noetic/setup.bash
cd /home/ubuntu/cage_uav_ws
catkin_make
source devel/setup.bash
```

有桌面显示时启动 Gazebo + RViz + Web：

```bash
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

纯 SSH/headless 验收把 `start_rviz:=true` 改为 `start_rviz:=false`。当前 90-3 正运行该 headless 版本，Web 端口为 8081。

启动任务：

```bash
curl -sS -X POST http://127.0.0.1:8081/api/scout/start \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id":"scout-my-run-001",
    "cage_id":0,
    "survey_relative_alt":25.0,
    "auto_rtl":true,
    "auto_land":true
  }'
```

查看状态：

```bash
watch -n 1 'curl -sS http://127.0.0.1:8081/api/status | python3 -m json.tool'
```

## 6. 尚未在本轮验证的范围

- K9 真机、真实相机检测、真实海况和 H12 实机接管仍需单独台架/海试；
- 本轮使用 `oracle_noisy` 作为仿真观测来源，但路线和在线地图更新没有读取 GT；
- 当前服务器以 headless Gazebo 运行，因此本轮没有在 90-3 桌面打开 RViz；Gazebo ROS 状态与 MAVROS pose 已做数值同步验证；
- 90-3 的用户级 `setuptools` 与 Ubuntu 20.04 系统 `importlib_metadata` 有版本冲突，所以构建/运行命令固定加入 `PYTHONNOUSERSITE=1`。
