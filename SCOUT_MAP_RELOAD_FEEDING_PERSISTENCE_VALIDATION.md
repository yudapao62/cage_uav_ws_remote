# Scout 历史地图持久化与 Feeding 验收记录

验收日期：2026-10-09  
目标主机：`ubuntu@90-3`  
工作区：`/home/ubuntu/cage_uav_ws`

## 1. 最终运行状态

- Gazebo、RViz、Web、mock MAVROS、Scout、Feeding、registry 均在运行。
- `/mavros/local_position/pose` 实测约 `20 Hz`。
- `/scout/lattice_map` 唯一发布者：`/scout_map_registry`。
- `/scout/working_lattice_map` 唯一发布者：`/scout_lattice_node`。
- Web：远端 `http://127.0.0.1:8081/`；现有本地 SSH 转发入口为 `http://127.0.0.1:18081/`。

## 2. 历史 v1 地图验收

任务：`scout-web-1791526104020`

```text
map_uuid       = 7dedb190-3d6e-4d0d-b01c-5cb04ce549c0
map_version    = 472
map_sha256     = 9507a9383674937ce3cc73770401400656172d3c0552f2a8a8d653cb2ae628e9
manifest_sha256= 83e021d50cdc3c69842b335dc749396deeb146ec7ee61bae1ac508c414c11de2
coverage       = 60/60 = 1.0
```

- legacy profile 同时绑定 WGS84 reference、`medium/42`、environment ID 和 ground-truth 文件 SHA-256。
- Web 列表、严格验证、显式激活均通过。
- 激活后等待 `202.7 s` 再 Preview，仍生成 60 个目标，不再受旧 30 秒 message-header 门限制。
- selector 写入后完整重启，恢复同一 UUID/version/map hash/manifest hash。
- 错 UUID 激活返回 `MAP_HASH_MISMATCH`，旧 active 保持不变。
- registry 心跳停止后 Preview 返回 `REGISTRY_HEARTBEAT_STALE`，按 fail-closed 拒绝。

## 3. Feeding 完整任务验收

任务：`feed-history-map-run-002`，目标 `R03C05`、`R03C06`。

- `/feeding/target_cells`、`/feeding/task_status`、`/mission/request` 均携带四项地图 pin。
- 运行期间竞争 lease 返回 `MAP_LEASE_CONFLICT`。
- 实际状态序列：

```text
FEEDING R03C05 feeder=true
FEEDING R03C05 feeder=false BETWEEN_CELL_TARGETS
FEEDING R03C06 feeder=true
RTL feeder=false
LAND feeder=false
FINISHED feeder=false
```

- 最终 `TASK_COMPLETED`、`FINISHED`、自动返航、降落、Feeder 关闭；终态后 lease 可重新获取，证明原任务 lease 已释放。

## 4. 新 schema v2 与自动激活验收

任务：`scout-v2-validation-001`

```text
map_uuid       = a942d5cf-0306-44d5-a956-858c23fb8b00
map_version    = 472
map_sha256     = 02e01ef3bb2202fb0f4dc275c2850a27e8c1f7a1c2fbe89758872e9f5785013e
manifest_sha256= a7ae2a8cb57ca1df4a12e5c66f309d4c3695a1ce79351a5f9e07d9e19f66255a
coverage       = 60/60 = 1.0
legacy flags   = false / false
```

- Scout 运行期间历史 active 地图保持不变。
- Scout 完成 60/60 建图、自动 RTL、自动降落、解除武装。
- v2 artifact 包含 reference、nominal/environment geometry、mapping config、software source、runtime config 和明确 wall/ROS 时间字段。
- manifest 写完后通过同一个 `/scout/activate_map` 服务严格校验并自动激活，没有旁路。
- v1 与 v2 的 `map_version` 同为 472，但 UUID 和两项内容 hash 不同；全链路 pin 能正确区分。
- 再次完整重启后，selector 恢复 v2 地图；`R03C05/R03C06` Preview 成功且携带 v2 四项 pin。

## 5. 构建与测试

- `catkin_make -j4` 通过。90-3 需要 `PYTHONNOUSERSITE=1`，以隔离主机用户目录中不兼容的 setuptools/importlib_metadata。
- 通过的测试包括 persistent map、repository、Feeder filter、target adapter、identity gate、Mission FSM 和 Scout Web API。
- repository 用例覆盖成功 v2、内容篡改、跨文件 version 不一致、非冻结地图和路径穿越。

## 6. 当前启动命令

```bash
cd /home/ubuntu/cage_uav_ws
source /opt/ros/noetic/setup.bash
export PYTHONNOUSERSITE=1
catkin_make -j4
source devel/setup.bash

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

若更换 world 或 ground-truth 文件，必须同时传入对应的 `environment_id` 和 `environment_geometry_sha256`，不能沿用本记录中的 medium/42 环境身份。
