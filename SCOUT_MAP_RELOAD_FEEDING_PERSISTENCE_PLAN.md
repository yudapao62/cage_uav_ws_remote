# 历史 Scout 地图重新加载与 Feeding 长期复用实施计划

> 状态：Codex 实施规范，后续编码必须以本文为准  
> 日期：2026-10-09  
> 本地工程：`/Users/yudapao/cage_uav_ws_remote`  
> 目标主机：`ubuntu@90-3:/home/ubuntu/cage_uav_ws`  
> 上位设计：`SCOUT_WEB_MISSION_IMPLEMENTATION_PLAN.md`、`SCOUT_FEEDER_IRREGULAR_LATTICE_SIMULATION_PLAN.md`

## 0. 一句话目标

把已经成功完成并落盘的 Scout 任务变成可校验、可列出、可显式激活、重启后可恢复、长期供 Feeding 使用的冻结地图；Feeding 全程固定使用同一个 `(map_uuid, map_version, map_sha256, manifest_sha256)`，任何损坏、错网箱、错坐标参考、运行中换图或身份地图不一致都必须 fail-closed，禁止投喂。

## 1. 本文解决什么

### 1.1 用户要求

1. 历史 Scout 结果不能只能查看 JSON，必须能重新加载到 ROS 运行系统。
2. 加载后的地图必须能长期供 Feeding 选择 cell、规划航线和做身份确认。
3. ROS、Gazebo 或整台主机重启后，不需要重新飞一遍 Scout 才能继续 Feeding。
4. 网页能够看到历史 Scout 地图列表、校验结果和当前激活地图，并能安全切换。
5. 不能通过简单增大 30 秒超时、循环修改时间戳或直接信任某个 `map.json` 来伪装完成。

### 1.2 不在本轮范围内

- 不重新设计 Scout 视觉关联算法。
- 不改变 `RxxCxx` 的 row-column 身份定义。
- 不让 Web 提交服务器任意绝对路径。
- 不允许 Feeding 飞行中热切换地图。
- 不自动选择“目录时间最新”的地图；第一次激活必须显式指定，之后只恢复上次已经成功激活的 selector。
- 不把哈希校验描述成防篡改签名。现有 SHA-256 只能发现文件损坏或普通不一致，不能防止有权限的人同时修改文件和 manifest。

## 2. 当前已验证的基线

### 2.1 已有底层能力，必须复用

当前工程不是完全没有恢复能力：

- `src/cage_grid/src/cage_grid/persistent_map.py`
  - 已有原子 JSON 保存；
  - 已有 `PersistentLatticeMap.load_json()`；
  - 能恢复 `map_uuid`、`map_version`、cell、协方差、`frozen` 和 `finalized_at`；
  - 已有基础 reload 单元测试。
- `src/cage_grid/scripts/scout_lattice_node.py`
  - 已有 `/persistent_lattice/restore_map_file`；
  - 节点启动时可以读取单个 `map.json`；
  - 会通过 latched `/scout/lattice_map` 发布恢复结果。
- `src/cage_simulation/scripts/scout_task_manager_node.py`
  - 已生成 `request/route/map/quality/result/events/manifest/bag`；
  - manifest 已记录文件 SHA-256。
- `src/cage_feeding_task/src/cage_feeding_task/persistent_map_adapter.py`
  - 已能把确认后的 cell 中心转成 Feeding target；
  - FULL_GRID、ROW_OR_COLUMN、SELECTED_CELLS 不需要重写。
- Feeding 当前任务内的 `task_map_model` 已经是固定快照；重复轮次沿用同一对象，这一点保留。

### 2.2 当前成功样例

第一份必须纳入端到端验收的历史任务：

```text
task_id:       scout-web-1791526104020
remote path:   /home/ubuntu/.ros/cage_uav/scout_runs/scout-web-1791526104020
map_uuid:      7dedb190-3d6e-4d0d-b01c-5cb04ce549c0
map_version:   472
cage_id:       0
confirmed:     60 / 60
coverage:      1.0
frozen:        true
terminal:      COMPLETED
mapping:       SUCCESS
flight:        SUCCESS
```

该任务是 schema v1 历史样例。它的 ENU 地理参考没有完整写进 `map.json`，所以只能按本文的 legacy 仿真兼容规则导入，不能在真实飞行配置中静默假定参考原点。

### 2.3 当前为什么不能长期使用

现状存在以下缺口：

1. `restore_map_file` 只能在节点启动时指定，正常 launch 没有暴露参数，也没有运行时 activate service。
2. 没有历史任务列表、当前 active map 状态和 Web 激活接口。
3. loader 没有验证 `manifest.json`、`result.json`、`quality.json` 的完整性与跨文件一致性。
4. 没有强制要求 `COMPLETED + mapping SUCCESS + frozen + coverage 合格`。
5. `PersistentLattice.msg` 不能表达 source task、frozen、finalized、activated、artifact hash 和完整地理参考。
6. Feeding 和身份滤波只携带 `map_version`，没有携带 `map_uuid`。两个不同地图都可能是 version 472。
7. 新 Scout reset 会直接替换节点当前 lattice；如果新任务失败，旧的可用地图无法作为独立 active map 保留。
8. `configuration_sha256` 目前对整个 `/persistent_lattice` 参数树求哈希，包含 restore 路径和消费侧参数，不能作为严格的地图兼容键。
9. 现有 `geometry_sha256` 只描述规则名义网格，不足以证明当前不规则物理网箱与历史 Scout 所在环境相同。
10. ENU 参考经纬高未完整进入历史 map 契约，配置变化后可能把正确 ENU 点转换成错误 WGS84 航点。

### 2.4 两个独立的 30 秒门

必须同时修改，不能只修其中一个：

1. `feeding_task_manager_node.py`
   - 用 `/persistent_lattice/task_map_max_age_sec=30` 检查 `/scout/lattice_map.header.stamp`；
   - 超时返回 `SCOUT_MAP_STALE`。
2. `feeder_identity_node.py` / `feeder_filter.py`
   - 用 `/persistent_lattice/feeder/stale_map_sec=30` 再检查一次同一时间戳；
   - 超时产生 `STALE_MAP`，mission FSM 会关闭投喂门。

启动恢复时给消息写一次“现在”的时间，只能再使用 30 秒。定时重发并刷新 header 仍然是错误实现，因为它掩盖了地图真实测量时间。

## 3. 强制设计原则

后续实现不得偏离以下原则：

1. **working map 与 active map 分离**  
   Scout 正在构建的是 working map；Feeding 只能消费已经完整验证并冻结的 active map。
2. **失败不替换旧图**  
   新 Scout 失败、取消、覆盖率不足或激活失败时，旧 active map 必须原样保留。
3. **身份与内容同时锁定**  
   UUID/version 是地图逻辑身份，map/manifest SHA-256 是内容 pin。所有下游均携带四项，不能只比较 version，也不能只信任 UUID/version 相同但内容 hash 已变化的目录。
4. **先完整校验，再原子切换**  
   所有文件和结构校验在候选对象上完成；任何一步失败都不能改变当前 active map。
5. **禁止任意路径输入**  
   Web/ROS 请求只接受安全 `task_id` 和预期 UUID/version/map hash/manifest hash；服务端在固定 `output_root` 中解析。
6. **激活期间互斥**  
   Scout、Feeding 或通用 mission 活跃、Feeder 已开启时不得切图。
7. **冻结地图不因 latched 消息超过 30 秒失效**  
   节点健康、激活时间、地图测量时间必须使用不同字段和不同策略。
8. **真实地图年龄不能被重发清零**  
   source/snapshot age 始终来自产物的墙钟完成时间。
9. **重启恢复仍要重新校验**  
   `active_map.json` selector 只说明用户上次选择了什么，不是可信地图本身。
10. **默认不猜“最新地图”**  
    selector 不存在或失效时保持 `NO_ACTIVE_MAP`，不扫描后静默选一个。
11. **Feeding fail-closed**  
    map registry 掉线、身份或内容 hash 不一致、地图不兼容或 cell 身份未确认时禁止投喂，但飞行安全恢复/RTL 仍由 mission FSM 处理。
12. **用 registry lease 关闭并发竞态**  
    Feeding 接单与地图激活必须在 registry 内竞争同一把租约/提交锁；只订阅状态再检查一次不能消除跨节点 TOCTOU。

## 4. 目标架构

```text
Scout observations
       |
       v
ScoutLatticeNode
  working_lattice
       |
       +---- /scout/working_lattice_map  (可变，仅 Scout 使用)
       |
       +---- finalize -> scout_runs/<task_id>/map.json
                                |
                                v
                    ScoutMapRepository/Validator
                                |
                 list + validate + activate + audit
                                |
                                v
                       ScoutMapRegistryNode
                         active_lattice
                  +-------------+--------------+
                  |                            |
                  v                            v
       /scout/lattice_map             /scout/active_map_status
       (冻结、latched、兼容名)          (1 Hz 健康与激活状态)
                  |
          +-------+-------------------+
          |                           |
          v                           v
 FeedingTaskManager            FeederIdentityNode
 固定身份+内容hash             GPS+视觉+地图身份
          |                           |
          +------------+--------------+
                       v
               Mission FSM identity gate
                       |
               confirmed 且地图一致
                       v
                   Feeder ON
```

### 4.1 Topic 语义

- `/scout/working_lattice_map`
  - 由 `scout_lattice_node.py` 发布；
  - 允许随观测增长；
  - 只供 ScoutTaskManager、Scout 页面进度和调试可视化使用；
  - 不允许 FeedingTaskManager 或 FeederIdentityNode 订阅。
- `/scout/lattice_map`
  - 继续保留现有名字，但语义改成“当前 active 的冻结地图”；
  - 只由 `scout_map_registry_node.py` 发布；
  - latched；
  - 没有 active map 时不伪造空地图，由 status 明确报告 `NO_ACTIVE_MAP`。
- `/scout/active_map_status`
  - latched，并以 1 Hz 发布健康心跳；
  - 是判断 registry 在线、地图已验证和激活身份的唯一运行时状态源。

不得让 `scout_lattice_node.py` 和 registry 同时发布 `/scout/lattice_map`。

## 5. 新增代码与消息合同

### 5.1 新增纯 Python 仓库/验证器

新增：

```text
src/cage_grid/src/cage_grid/scout_map_repository.py
```

职责：

- 安全解析 `output_root/<task_id>`；
- 扫描可用任务目录；
- 校验 manifest 和所有登记文件 hash；
- 校验 result、quality、map 跨文件一致性；
- 调用并加强 `PersistentLatticeMap.load_json()`；
- 返回不可变的 `ValidatedScoutMap`；
- 不依赖 ROS，便于完整单元测试。

建议数据对象：

```python
ValidatedScoutMap(
    task_id,
    cage_id,
    map_uuid,
    map_version,
    map_sha256,
    manifest_sha256,
    frozen,
    source_created_at_unix_sec,
    source_completed_at_unix_sec,
    confirmed_cells,
    expected_cells,
    coverage,
    frame_id,
    reference_wgs84,
    geometry_sha256,
    environment_geometry_sha256,
    observation_source,
    legacy_reference_assumed,
    legacy_environment_assumed,
    lattice,
)
```

### 5.2 新增 registry 节点

新增：

```text
src/cage_grid/scripts/scout_map_registry_node.py
```

该节点是 active map 的唯一所有者和唯一发布者，负责：

- `/scout/list_maps`；
- `/scout/activate_map`；
- `/scout/lattice_map`；
- `/scout/active_map_status`；
- `/scout/manage_map_lease`；
- 原子保存 `${output_root}/.active_map.json`；
- 保存 `${output_root}/map_activation_events.jsonl`；
- 启动时重新校验 selector 指向的完整产物并恢复；
- 订阅 Scout、Feeding、mission、Feeder 状态，执行切图互斥。

### 5.3 扩展 `PersistentLattice.msg`

保留现有字段并增加：

```text
string source_task_id
bool frozen
time source_created_at_ros
time source_finalized_at_ros
time activated_at_ros
float64 source_created_at_unix_sec
float64 source_completed_at_unix_sec
float64 activated_at_unix_sec
string map_sha256
string manifest_sha256
string mapping_config_sha256
string software_source_sha256
string observation_source
float64 reference_lat
float64 reference_lon
float64 reference_alt
string reference_sha256
string environment_geometry_sha256
uint32 confirmed_cells
uint32 expected_cells
float32 coverage
bool restored_from_history
bool legacy_reference_assumed
bool legacy_environment_assumed
```

时间要求：

- `header.stamp`：本次 ROS 发布时刻，只供 ROS 观测；
- `source_*_ros`：原任务使用的 ROS/Gazebo 时钟，只保存 provenance；
- `*_unix_sec`：UTC Unix 墙钟秒，用于跨重启审计和 source age；
- schema v1 的 `map.created_at/finalized_at` 是 ROS/Gazebo 时间，不能当 UTC；其 `source_completed_at_unix_sec` 取 `manifest.finalized_at`，并可与 `result.finished_at` 做容差复核；
- heartbeat 超时不能用以上任意 artifact 时间计算，必须用消费者收到回调时记录的单调时钟。

### 5.4 新增 `ScoutMapSummary.msg`

```text
uint8 VALIDATION_UNKNOWN=0
uint8 VALIDATION_FULL_VALID=1
uint8 VALIDATION_INVALID=2
uint8 COMPATIBILITY_UNKNOWN=0
uint8 COMPATIBILITY_COMPATIBLE=1
uint8 COMPATIBILITY_INCOMPATIBLE=2

string task_id
int32 cage_id
string map_uuid
uint64 map_version
string map_sha256
string manifest_sha256
float64 source_completed_at_unix_sec
uint32 confirmed_cells
uint32 expected_cells
float32 coverage
bool frozen
uint8 validation_state
uint8 compatibility_state
bool active
bool legacy_reference_assumed
bool legacy_environment_assumed
float64 last_validated_at_unix_sec
string validation_code
string validation_message
```

### 5.5 新增 `ScoutMapActivationStatus.msg`

```text
uint8 STATUS_NO_ACTIVE=0
uint8 STATUS_ACTIVE=1
uint8 STATUS_INVALID=2
uint8 STATUS_SOURCE_EXPIRED=3
uint8 STATUS_REGISTRY_ERROR=4

std_msgs/Header header
uint8 status
string task_id
int32 cage_id
string map_uuid
uint64 map_version
string map_sha256
string manifest_sha256
float64 source_completed_at_unix_sec
float64 activated_at_unix_sec
float64 last_validated_at_unix_sec
uint32 confirmed_cells
uint32 expected_cells
float32 coverage
bool frozen
bool compatible
bool legacy_reference_assumed
bool legacy_environment_assumed
string reason_code
string message
string last_attempt_task_id
float64 last_attempt_at_unix_sec
bool last_attempt_success
string last_attempt_reason_code
string last_attempt_message
```

`last_validated_at_unix_sec` 是最近一次完整 artifact 校验时刻，不表示 registry 每秒重新读取和 hash 全部文件。1 Hz heartbeat 只重发内存中的不可变验证结论。

### 5.6 新增 `ActivateScoutMap.srv`

```text
string task_id
int32 cage_id
string expected_map_uuid
uint64 expected_map_version
string expected_map_sha256
string expected_manifest_sha256
---
bool success
bool committed
string reason_code
string message
string map_uuid
uint64 map_version
string map_sha256
string manifest_sha256
uint32 confirmed_cells
uint32 expected_cells
float32 coverage
float64 source_completed_at_unix_sec
float64 activated_at_unix_sec
bool idempotent
bool legacy_reference_assumed
bool legacy_environment_assumed
```

请求不得包含绝对路径。四个 `expected_*` 字段均必填；version 必须大于 0。UUID/version 防止选错逻辑地图，map/manifest hash 防止列表刷新后目录内容被同身份替换。

### 5.7 新增 `ListScoutMaps.srv`

```text
int32 cage_id
bool include_invalid
---
bool success
string reason_code
string message
cage_msgs/ScoutMapSummary[] maps
```

`cage_id=-1` 表示所有可见网箱。列表也必须做完整校验后才返回 `VALIDATION_FULL_VALID`；可以按 manifest hash、文件 stat 和策略配置 hash 做缓存，但 activate 必须重新读取 bytes、重新 hash、重新解析，不能复用列表缓存。某个损坏目录不能阻止其他有效目录返回。

### 5.8 下游地图身份字段

修改：

```text
src/cage_msgs/msg/CellTargetArray.msg
src/cage_msgs/msg/FeederCellHypothesisArray.msg
src/cage_msgs/msg/FeederIdentityState.msg
src/cage_msgs/msg/FeedingTaskStatus.msg
src/cage_msgs/msg/MissionRequest.msg
```

至少增加：

```text
string map_uuid
uint64 map_version
string map_sha256
string manifest_sha256
```

`CellTargetArray.msg` 还必须增加 `string task_id`，不能只依赖会在进程重启后重新计数的 `header.seq`。`FeederIdentityState.msg` 还要增加：

```text
string task_map_uuid
uint64 task_map_version
string task_map_sha256
string task_manifest_sha256
uint8 STATUS_MAP_MISMATCH=4
```

`MissionRequest.msg` 必须携带 `task_map_uuid/version/map_sha256/manifest_sha256`。Mission FSM 接受 Feeding mission 时保存这四项预期值，并同时核对 `FeederIdentityState` 的 active map 和 task map 字段；仅靠 identity 节点自行判断不构成完整安全闭环。`FeedingTaskStatus.msg` 同步暴露任务绑定地图，供 Web 和审计使用。

Feeding 生成 target 时必须复制完整四项。Feeder identity 当前使用的 active map 与任务固定地图任一项不一致时发布 `STATUS_MAP_MISMATCH`，FSM 必须关闭投喂门。

### 5.9 新增 `ManageScoutMapLease.srv`

```text
uint8 OP_ACQUIRE=0
uint8 OP_RENEW=1
uint8 OP_RELEASE=2

uint8 operation
string feeding_task_id
string lease_id
string map_uuid
uint64 map_version
string map_sha256
string manifest_sha256
float32 requested_ttl_sec
---
bool success
string reason_code
string message
string lease_id
float64 expires_in_sec
```

FeedingTaskManager 必须在接受任务、发布 mission request 之前 acquire；运行中按墙钟定期 renew；完成、取消和失败后 release。Registry 在同一锁内串行处理 lease acquire 与 map activation：存在有效 lease 时拒绝切图；activation 已提交后旧地图的 lease acquire 也必须拒绝。租约超时使用 registry 的单调时钟，不能使用 `/clock`。

## 6. 产物与完整性校验

### 6.1 安全路径

激活请求只接受符合以下规则的 task ID：

```text
^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$
```

服务端执行：

1. `candidate = realpath(join(output_root, task_id))`；
2. 使用 `commonpath` 确认 candidate 位于 `realpath(output_root)` 内；
3. 拒绝越界、缺失目录和任何 task/artifact symlink；
4. 要求任务目录是目录、artifact 是普通文件；
5. manifest 的文件键只允许固定 basename allowlist，拒绝绝对路径、`..` 和任意路径分隔符；
6. 每个 artifact 打开后仍要用 `realpath/commonpath` 复核位于 task 目录；
7. 不把内部绝对路径返回 Web。

### 6.2 必需文件

至少要求：

```text
manifest.json
map.json
quality.json
result.json
request.json
```

对 `manifest.files` 中登记的每一个文件重新计算 SHA-256。文件缺失、长度异常或任意 hash 不一致均拒绝激活。

另外强制：

- 上述必需文件必须出现在 `manifest.files` 中，不能只因磁盘上存在就接受；
- 设定目录文件数、单文件大小、JSON 大小、cell/edge 数量上限；
- hash 与 JSON 解析必须针对同一次打开后读取到的同一份 bytes，避免“先 hash、替换文件、再 parse”的 TOCTOU；
- JSON parser 必须拒绝重复 key、NaN、Infinity 和尾随的额外 JSON 内容；
- manifest 登记的可选 bag/events 等文件也必须 hash 正确；未生成的可选文件不得登记为空路径。

### 6.3 跨文件一致性

不能要求 v1 每个文件都具有同一组字段，也不能要求各文件 `schema_version` 数字永远相同。v1 按以下来源矩阵比较；字段缺失只在表中明确允许的文件里兼容：

| 字段 | v1 权威/对照来源 |
|---|---|
| `task_id` | 目录名、request、map、quality、result、manifest 全部一致 |
| `cage_id` | request、map、result 一致；quality/manifest v1 缺失是允许的 |
| `map_uuid/map_version` | map、quality、`result.map`、manifest 一致；request v1 不包含 |
| `geometry_sha256` | map 顶层与 map.metadata 一致，再与 `result.map` 对照；quality/manifest/request v1 不包含 |
| `confirmed_cells` | 从 map cells 按 confirmed 条件重新计算，再与 quality、`result.map` 对照 |
| `expected_cells` | 从 `rows*cols` 推导，再与 quality、`result.map` 对照 |
| `coverage` | 用派生的 confirmed/expected 计算，再与 quality、`result.map` 对照 |
| source 完成墙钟 | manifest.finalized_at 为 v1 主来源，与 result.finished_at 做合理顺序/容差检查 |
| Scout 配置 | manifest 的 preset、seed、observation_source 与 legacy profile/当前策略比较 |

每个文件独立维护 `map_schema_version`、`manifest_schema_version`、`result_schema_version`、`quality_schema_version`。若 v2 需要统一整个产物包，再单独增加 `artifact_bundle_schema_version`，不要复用各文件自己的 schema 字段。

并强制：

```text
result.terminal_status == "COMPLETED"
result.mapping_outcome == "SUCCESS"
result.map.frozen == true
map.frozen == true
quality.coverage >= minimum_coverage
map coverage == confirmed_cells / expected_cells
```

浮点 coverage 使用明确容差比较，例如 `1e-6`。

### 6.4 地图结构和数值

验证器必须额外检查：

- UUID 非空且格式有效；
- 所有 SHA-256 都是 64 位小写十六进制；
- version 大于 0；
- `min_confirmations >= 1`；
- rows、cols 与目标网箱配置一致；
- `expected_cells == rows * cols`；
- `(row,col)` 唯一且在范围内；
- `cell_id == RxxCxx` 且唯一；
- 不允许后出现的重复 cell 静默覆盖前一条；
- 所有坐标、时间、置信度均为有限数值；
- `identity_confidence` 在 `[0,1]`；
- `last_seen >= first_seen`；
- `observation_count >= confirmation_count >= 0`；
- `covariance_xy` 恰好 4 项、近似对称、对角非负且半正定；
- confirmed cell 的 `confirmation_count >= min_confirmations`；
- nominal position 必须在容差内等于当前名义模型对应 cell；
- `position_delta == estimated_position - nominal_position`；
- anchor 恰好 3 项且全部有限；
- reference 纬度在 `[-90,90]`、经度在 `[-180,180]`、高度有限；
- polygon 顶点数量、数值有限性、非退化面积和方向规则合法；
- edge 不能重复或指向不存在 cell，并且 edge 集合必须与当前已存在 cell 推导出的规范上下/左右邻接集合完全一致，不能只检查每条边“看起来合法”；
- map 顶层与 `metadata` 中重复字段必须完全一致；
- `created_at <= finalized_at` 只能在相同 clock domain 内比较；
- FULL_GRID 只允许在需要的全部 cell 均 confirmed 时生成；
- SELECTED_CELLS 至少逐个验证被选 cell confirmed。

通用结构/数值规则补进 `PersistentLatticeMap.load_json()`；`COMPLETED/frozen/quality/manifest` 等任务产物级规则放在 repository wrapper，避免破坏加载非冻结调试快照的现有用途。不能只依靠 Web 校验。

### 6.5 运行环境兼容性

必须检查：

- `cage_id`；
- `frame_id == map`；
- `anchor_definition` 与 `anchor_enu`；
- rows、cols、cell size、yaw；
- WGS84 reference lat/lon/alt 与 reference hash；
- 名义几何 hash；
- 当前仿真 world 的 environment geometry hash，或真实网箱的受控 cage asset/calibration ID；
- source map age 策略；
- observation source 是否被当前运行策略允许。

`observation_source` 默认是来源信息，不应仅因为当前 Feeder 使用另一种视觉源就拒绝；真实飞行可以通过 allowlist 明确限制。不能继续把“当前 observation_source 必须等于历史 observation_source”写死在 loader 中。

当前环境身份必须来自单一权威参数：

```text
/scout_map_registry/environment_geometry_sha256
/scout_map_registry/environment_id
```

仿真 launch 在生成/选择 world 和 ground-truth 文件时注入这两个值，registry 不得临时订阅 `/sim_gt/*` 来绕过算法与 GT 隔离。真实网箱通常无法证明网衣形变“完全没变”，因此 environment ID 只代表资产/校准版本；物理变化风险仍由 source age、协方差老化和重新 Scout 策略管理。

### 6.6 新产物 schema v2

未来产物 bundle v2 必须精确定义字段位置：

```text
map.json 顶层及 metadata（重复值必须一致）：
  reference_lat / reference_lon / reference_alt
  reference_sha256
  nominal_geometry_sha256
  environment_geometry_sha256
  environment_id
  mapping_config_sha256
  software_source_sha256
  source_created_at_ros_sec / source_finalized_at_ros_sec
  source_created_at_unix_sec / source_completed_at_unix_sec

manifest.json：
  artifact_bundle_schema_version
  map_schema_version / result_schema_version / quality_schema_version
  reference_sha256
  nominal_geometry_sha256
  environment_geometry_sha256 / environment_id
  mapping_config_sha256
  software_source_sha256 / software_commit
  runtime_config_sha256
  created_at_unix_sec / finalized_at_unix_sec
```

哈希拆分规则：

- `mapping_config_sha256`：只覆盖影响 Scout 建图结果的算法参数；
- `software_source_sha256/software_commit`：标识实际建图软件版本；现有 manifest 的 `source_sha256` 只 hash task manager 单文件，不能冒充完整算法代码 hash；
- `nominal_geometry_sha256`：行列、cell size、yaw、anchor；
- `environment_geometry_sha256`：实际不规则网箱/world 几何；
- `runtime_config_sha256`：restore path、Web、Feeder 等运行项，只记录 provenance，不作为地图几何兼容硬条件。

所有 hash 的输入都使用文档化的 canonical JSON：UTF-8、键排序、紧凑分隔符、SI 单位、禁止 NaN/Inf，并固定浮点规范化方法。时间统一使用 `float64 *_unix_sec`（UTC Unix 秒）和明确命名的 `*_ros_sec`；不得使用含糊的 `created_at_utc` 字符串。v1 的 `geometry_sha256` 迁移解释为 `nominal_geometry_sha256`，不能冒充实际 environment hash。

## 7. 历史 schema v1 兼容策略

`scout-web-1791526104020` 缺少完整 WGS84 reference 和 environment geometry hash。为了既能验证现有仿真结果，又不把风险带入真实飞行，采用以下规则：

1. `/scout_map_registry/allow_legacy_v1` 默认：
   - 专用仿真 launch：`true`；
   - realflight launch：`false`。
2. 仿真 legacy 导入只能绑定当前只读配置中的：

   ```text
   /cage_grid/reference_lat
   /cage_grid/reference_lon
   /cage_grid/reference_alt
   ```

   Web 不得上传这些值。
3. 每个允许使用的 v1 地图必须存在管理员配置的 legacy compatibility profile，以 `map_sha256` 为键，至少绑定：

   ```text
   reference_lat/lon/alt
   manifest preset/seed
   environment_id
   当前 ground-truth/world artifact sha256
   nominal_geometry_sha256
   ```

   Registry 同时比较 manifest preset/seed 与当前仿真参数。这个 profile 是仿真专用弱兼容证明，不等于 v1 产物本身已经记录了实际环境 hash。
4. 仍要校验旧产物 manifest、result、quality、map、cage、frame、anchor、名义 geometry hash；profile、preset、seed 或当前 environment ID/hash 任一不一致都拒绝。
5. v1 source age 使用 `manifest.finalized_at` 墙钟值；`map.finalized_at` 只作为 Gazebo/ROS 时钟 provenance，绝不能用于跨重启年龄计算。
6. active status 和 Web 必须显示：

   ```text
   legacy_reference_assumed = true
   legacy_environment_assumed = true
   warning = LEGACY_REFERENCE_AND_ENVIRONMENT_BOUND_FROM_PROFILE
   ```

7. legacy 地图可以用于本项目仿真验收，但真实飞行必须先生成 schema v2 地图或执行受控离线迁移；不能隐藏警告直接使用。
8. 原始历史文件不得就地修改，否则会破坏 manifest。若以后提供迁移工具，输出必须写入独立 derived 目录并生成新的 manifest。
9. `PersistentLattice.msg` 扩展后 ROS1 message MD5 会改变；历史恢复以 JSON artifact 为准，不承诺旧 `mission.bag` 中该消息能按新定义直接回放。若确需旧 bag 回放，应提供旧消息兼容包或离线迁移工具。

## 8. 原子激活流程

`/scout/activate_map` 必须严格执行：

1. 校验 request 字段和安全 task path。
2. 读取并验证完整 artifact suite。
3. 构建独立候选 `ValidatedScoutMap` 和 ROS message，暂不改变 active。
4. 检查 `expected_map_uuid/version/map_sha256/manifest_sha256`。
5. 检查运行互斥状态：
   - Scout 为 IDLE/COMPLETED/FAILED/CANCELED/CANCELLED；
   - Feeding 为 TASK_IDLE/TASK_PREVIEW/TASK_COMPLETED/TASK_ERROR/TASK_CANCELLED；
   - mission 为 IDLE/FINISHED；
   - Feeder 为 OFF；
   - 不存在 finalize/activation in progress；
   - 不存在有效 Feeding map lease。
6. 在锁外生成 selector 临时文件，flush 并 fsync，但暂不替换正式 selector。
7. 获取 registry 锁；在同一临界区重新检查 active revision、运行状态和 lease。activate 与 Feeding lease acquire 必须在此串行，不能只靠跨节点状态 topic 消除 TOCTOU。
8. 若目标与当前 active 的 UUID/version/map hash/manifest hash 完全相同，返回成功且 `idempotent=true`。该幂等请求即使存在同图 lease 也可成功，但绝不能改变任何状态。
9. 准备内存 candidate 后执行 `os.replace(temp_selector, selector)`；**selector 原子替换成功是唯一 commit point**：

   ```json
   {
     "schema_version": 1,
     "task_id": "scout-web-1791526104020",
     "cage_id": 0,
     "map_uuid": "7dedb190-3d6e-4d0d-b01c-5cb04ce549c0",
     "map_version": 472,
     "map_sha256": "...",
     "manifest_sha256": "...",
     "activated_at_unix_sec": 1791527000.0
   }
   ```

10. commit 后立即切换内存 active，并 fsync selector 父目录；释放锁后发布 latched map/status、追加 audit。
11. commit 前任何失败均回滚内存 candidate，旧 selector/active 不变。
12. commit 后 publish、audit 或父目录 fsync 发生异常时不得假装“未激活”或回滚到旧图：进入 `REGISTRY_ERROR`，保留 selector 作为持久事实并幂等重试发布/审计；服务响应必须带 `committed=true` 和降级 reason。

commit 前失败：

- 返回稳定错误码；
- 不修改 selector；
- 不修改当前 active；
- 不发布候选地图；
- Web 保持显示原 active map。

磁盘、内存和 ROS 发布无法组成一个跨介质事务，因此禁止再宣称“commit 后任何失败仍能保证 selector 未修改”。

### 8.1 启动恢复

registry 启动时：

1. selector 不存在：发布 `NO_ACTIVE_MAP`，不自动选最新。
2. selector 存在：重新执行完整校验，不直接信任 selector。
3. 完整校验成功且 selector 中 UUID/version/map hash/manifest hash 全部匹配：恢复并发布。
4. 校验失败：发布 `INVALID`，保留 selector 供排障但不发布地图，不回退到另一个目录。
5. 启动阶段恢复 selector 是恢复同一持久事实，不是切换到另一张地图；允许在 mission 尚为 WAIT_FCU/WAIT_GPS 时完成，但只发布地图，不发任何飞控命令。真正切换仍执行完整运行互斥。

## 9. 时间与长期有效性

### 9.1 三种时间必须分开

| 时间 | 含义 | 用途 |
|---|---|---|
| `transport/status age` | registry 心跳最近是否到达 | 判断节点/ROS 链是否在线 |
| `activation age` | 这张图何时被本次运行激活 | 审计和 UI，不代表地图刚测量 |
| `snapshot/source age` | Scout 实际何时完成测量 | 物理几何寿命和协方差老化 |

实现细则：

- v1 source 完成墙钟取 `manifest.finalized_at`，不得使用约 191 秒之类的 `map.finalized_at` 仿真时钟；
- v2 使用显式 `source_completed_at_unix_sec`；
- Registry 不能只用 `rospy.Timer` 发心跳，因为 `/use_sim_time` 暂停或回拨时它也会停；应使用 wall-clock 线程或等价的 wall timer；
- 每个消费者在 status 回调到达时记录本进程的 `time.monotonic()`，heartbeat age 由该值计算，不用 `header.stamp` 相减；
- 系统墙钟向后调整时，source age 进入 `CLOCK_INVALID`/保守拒绝策略，不能得到负年龄后当作“更新”。

### 9.2 参数替换

废弃把 map message header 当作 source age 的做法：

```text
/persistent_lattice/task_map_max_age_sec
/persistent_lattice/feeder/stale_map_sec
```

替换为：

```yaml
scout_map_registry:
  active_status_timeout_sec: 3.0
  max_snapshot_age_sec: 0.0       # 仿真：0 表示不限制
  heartbeat_rate_hz: 1.0
  allow_legacy_v1: true

persistent_lattice:
  feeder:
    source_age_variance_rate_m2_per_sec: 0.0  # 仿真固定几何
```

真实飞行必须提供明确、非零的 `max_snapshot_age_sec` 和适合网箱变化速度的协方差老化策略；不能照搬仿真的无限寿命。

### 9.3 Feeding 接单门

FeedingTaskManager 应检查：

- active status 心跳未超时；
- status 为 ACTIVE；
- map frozen、compatible、coverage 合格；
- status UUID/version/hash 与收到的 active map 完全一致；
- source age 未超过环境策略；
- target cage 一致。

`PersistentMapTargetModel.from_message()` 必须从已经验证的 active map 消息读取并冻结 reference、UUID、version、map hash、manifest hash、frozen 和 coverage；不得再从当前 `/cage_grid/reference_*` 参数另传一个可能不同的 reference。

不再因为 latched map 超过 30 秒未重发而拒绝。

### 9.4 身份滤波门

FeederIdentityNode 应：

- 使用 source age 做真实协方差老化；
- 使用 active status heartbeat 判断 registry 在线；
- 使用 GPS/视觉各自的时间戳判断观测新鲜度；
- 收到 active map 时按该图 reference 重建/切换 `LocalENU`，并与 active status 的 reference hash 对照；不得只在节点启动时读取一次 `/cage_grid/reference_*`；
- current active map 与 task map UUID/version/map hash/manifest hash 任一不一致时发布 `STATUS_MAP_MISMATCH`；
- 不通过刷新 map header 把 source age 归零。

## 10. Feeding 如何固定并使用地图

### 10.1 任务创建

用户 preview/start 时：

1. FeedingTaskManager 获取当前 active map；
2. 构造 immutable `PersistentMapTargetModel`；
3. 固定：

   ```text
   task_map_uuid
   task_map_version
   task_map_sha256
   task_manifest_sha256
   task_map_model
   ```

4. 向 registry acquire 对应四项的 map lease；只有 acquire 成功才能接受任务。
5. 生成 targets，并在 `CellTargetArray` 和 `MissionRequest` 中携带 task ID 与完整四项地图 pin；
6. Mission FSM 接受请求时保存四项预期值；
7. 任务结束前不得被后来收到的 active map topic 改写，并持续 renew lease。

### 10.2 飞行中

- Registry 因有效 lease 拒绝其他激活请求并返回 `MAP_SWITCH_BUSY`；任务完成/取消/失败后 FeedingTaskManager release lease。
- FeederIdentityNode 同时比较 active map 和 target map 身份。
- Mission FSM 只有在以下条件同时满足时允许 Feeder ON：

  ```text
  identity status == CONFIRMED
  identity cell_id == expected cell_id
  identity data 未超时
  active map UUID/version/map hash/manifest hash == task map 四项
  FSM 保存的 mission 预期四项 == identity state 的 task map 四项
  registry status == ACTIVE 且心跳正常
  ```

- 任意地图身份不一致立即关闭 Feeder，错误码 `TASK_MAP_MISMATCH`，再由 FSM 按现有安全路径处理。

### 10.3 Scout 与 active map

- 新 Scout 启动只 reset working map，不删除 active map。
- Scout 飞行期间 Feeding 仍因任务互斥不能启动，但旧 active map 保存在 registry。
- Scout 成功完成并写完 manifest 后，可以自动调用同一 `/scout/activate_map` 流程，但必须按以下顺序：`写 result/manifest -> 发布 terminal Scout status -> 离开 Scout manager 锁 -> 异步调用 activate`。不得在 manager 锁内同步调用，也不得在状态仍为 LAND/FINALIZING 时被 registry 当成 `SCOUT_BUSY`。
- 自动激活失败不修改旧 active；Scout result/manifest 已经固定，不得为了写激活失败再回写并破坏 hash。失败信息进入 active status 的 `last_attempt_*` 和 activation audit，Web 单独显示 `MAP_ACTIVATION_FAILED`，允许稍后重试而不重飞。
- Scout 失败或取消时绝不 promote working map。

## 11. Web API 与页面

### 11.1 新增接口

```http
GET /api/scout/maps?cage_id=0&include_invalid=false
GET /api/scout/maps/active
POST /api/scout/maps/activate
```

激活请求示例：

```json
{
  "task_id": "scout-web-1791526104020",
  "cage_id": 0,
  "expected_map_uuid": "7dedb190-3d6e-4d0d-b01c-5cb04ce549c0",
  "expected_map_version": 472,
  "expected_map_sha256": "<从列表响应原样复制>",
  "expected_manifest_sha256": "<从列表响应原样复制>"
}
```

成功返回 HTTP 200：

```json
{
  "success": true,
  "idempotent": false,
  "task_id": "scout-web-1791526104020",
  "cage_id": 0,
  "map_uuid": "7dedb190-3d6e-4d0d-b01c-5cb04ce549c0",
  "map_version": 472,
  "map_sha256": "...",
  "manifest_sha256": "...",
  "confirmed_cells": 60,
  "expected_cells": 60,
  "coverage": 1.0,
  "active": true,
  "legacy_reference_assumed": true
}
```

HTTP 规则：

- `400`：JSON 或字段非法；
- `404`：task/artifact 不存在；
- `409`：Scout/Feeding/mission/lease busy，或 expected identity/hash 不一致；
- `422`：artifact hash、质量、schema 或环境兼容校验失败；
- `503`：registry/service 不可用。

### 11.2 页面要求

Scout 区域新增“历史地图”表格，显示：

- task ID；
- cage ID；
- map UUID/version；
- map/manifest SHA-256；
- frozen 时间；
- confirmed/expected/coverage；
- validation state/compatibility state/最近完整校验时间；
- schema/legacy 警告；
- 是否 active；
- 激活按钮。

还要显示 `last_attempt_task_id/outcome/reason`，这样旧 active 仍健康时，页面也能表达“刚完成的新 Scout 自动激活失败”，不能只展示当前 active 的 ACTIVE 状态。

页面必须明确区分：

- 当前 Scout working map；
- 当前 Feeding active map；
- 上一次 Scout task status。

不能继续把 `scout_status.map_uuid` 和后续人工激活的历史地图显示成同一个概念。

## 12. 稳定失败码

至少实现并测试：

```text
INVALID_TASK_ID
ARTIFACT_NOT_FOUND
PATH_OUTSIDE_OUTPUT_ROOT
MANIFEST_MISSING
MANIFEST_INVALID
ARTIFACT_HASH_MISMATCH
SCHEMA_UNSUPPORTED
TASK_ID_MISMATCH
CAGE_ID_MISMATCH
MAP_UUID_MISMATCH
MAP_VERSION_MISMATCH
MAP_HASH_MISMATCH
MANIFEST_HASH_MISMATCH
MAP_NOT_FROZEN
MAP_QUALITY_REJECTED
MAP_STRUCTURE_INVALID
FRAME_MISMATCH
ANCHOR_MISMATCH
REFERENCE_MISSING
REFERENCE_MISMATCH
GEOMETRY_MISMATCH
SOURCE_MAP_EXPIRED
CLOCK_INVALID
OBSERVATION_SOURCE_DISALLOWED
SCOUT_BUSY
FEEDING_BUSY
MISSION_BUSY
FEEDER_ACTIVE
MAP_SWITCH_BUSY
MAP_LEASE_CONFLICT
MAP_LEASE_EXPIRED
TASK_MAP_MISMATCH
SELECTOR_WRITE_FAILED
NO_ACTIVE_MAP
REGISTRY_HEARTBEAT_STALE
LEGACY_REFERENCE_NOT_ALLOWED
LEGACY_ENVIRONMENT_NOT_ALLOWED
LEGACY_PROFILE_MISMATCH
```

错误码稳定、供程序判断；message 可给人阅读，不能让前端解析 message 文本。

## 13. 需要修改的文件

### 13.1 新增

```text
src/cage_grid/src/cage_grid/scout_map_repository.py
src/cage_grid/scripts/scout_map_registry_node.py
src/cage_grid/test/test_scout_map_repository.py
src/cage_grid/test/test_scout_map_registry.py
src/cage_msgs/msg/ScoutMapSummary.msg
src/cage_msgs/msg/ScoutMapActivationStatus.msg
src/cage_msgs/srv/ActivateScoutMap.srv
src/cage_msgs/srv/ListScoutMaps.srv
src/cage_msgs/srv/ManageScoutMapLease.srv
src/cage_web_bridge/test/test_scout_map_web_api.py
```

### 13.2 修改

```text
src/cage_msgs/CMakeLists.txt
src/cage_msgs/package.xml
src/cage_msgs/msg/PersistentLattice.msg
src/cage_msgs/msg/CellTargetArray.msg
src/cage_msgs/msg/FeederCellHypothesisArray.msg
src/cage_msgs/msg/FeederIdentityState.msg
src/cage_msgs/msg/FeedingTaskStatus.msg
src/cage_msgs/msg/MissionRequest.msg

src/cage_grid/src/cage_grid/persistent_map.py
src/cage_grid/src/cage_grid/feeder_filter.py
src/cage_grid/scripts/scout_lattice_node.py
src/cage_grid/scripts/feeder_identity_node.py
src/cage_grid/config/persistent_lattice.yaml
src/cage_grid/CMakeLists.txt

src/cage_simulation/scripts/scout_task_manager_node.py
src/cage_simulation/launch/persistent_lattice_sim.launch
src/cage_simulation/config/persistent_lattice_sim.yaml
src/cage_simulation/config/scout_mission.yaml

src/cage_feeding_task/src/cage_feeding_task/persistent_map_adapter.py
src/cage_feeding_task/scripts/feeding_task_manager_node.py
src/cage_feeding_task/test/test_persistent_map_adapter.py

src/cage_mission_fsm/src/cage_mission_fsm/identity_gate.py
src/cage_mission_fsm/scripts/mission_fsm_node.py
src/cage_mission_fsm/test/test_identity_gate.py

src/cage_web_bridge/scripts/web_bridge_node.py
src/cage_web_bridge/www/index.html
src/cage_web_bridge/config/web.yaml

src/cage_bringup/scripts/persistent_lattice_event_logger.py
```

如 launch/bag topic 列表中仍只有 `/scout/lattice_map`，还要加入 `/scout/working_lattice_map` 和 `/scout/active_map_status`。

`/scout_mission/output_root` 必须成为 ScoutTaskManager 与 registry 共用的单一权威目录参数；禁止两个节点各自使用不同默认值。最终实现中，working 节点的 `restore_map_file` 标记 deprecated 并从正常 launch 移除，启动恢复只能由 registry selector 负责。

## 14. 分阶段实施顺序

### Phase 0：先写测试合同

1. 为 repository validation 写失败用例。
2. 为 UUID/version/map hash/manifest hash 绑定和 map lease 写 adapter/identity/FSM 用例。
3. 为超过 60 秒仍能 preview 写时间语义用例。
4. 测试先失败，再进入实现；不得先删除旧 stale gate 让测试假通过。

### Phase 1：严格 artifact repository

1. 抽取 manifest hash 逻辑复用，不复制散落代码。
2. 实现路径限制、artifact suite、跨文件和数值校验。
3. 加强 `load_json()` 的重复 cell、范围和有限数检查。
4. 完成 schema v1 仿真兼容和 schema v2 契约。

完成条件：纯 Python 单测覆盖成功任务、损坏文件和全部主要拒绝路径。

### Phase 2：working/active 双缓冲和 registry

1. `scout_lattice_node.py` 改发 `/scout/working_lattice_map`。
2. 新 registry 成为 `/scout/lattice_map` 唯一发布者。
3. 实现 list/activate、selector、audit、启动恢复。
4. 激活失败保留旧 active。

完成条件：两个节点不存在同 topic 多发布者；新 Scout reset 不影响旧 active。

### Phase 3：重构时间语义

1. 增加 active status heartbeat。
2. Feeding 接单门不再用 map header 的 30 秒 age。
3. identity filter 分开 registry heartbeat 和 source age。
4. heartbeat 使用回调接收的 monotonic 时间，并以 wall-clock 线程发布。
5. 仿真 source age 可无限；真实飞行必须显式配置。

完成条件：激活后等待至少 60 秒，preview/start 仍通过；registry 停止心跳后 fail-closed。

### Phase 4：地图身份、内容 hash 与 lease 全链路绑定

1. 扩展消息。
2. adapter 保存 reference、frozen、coverage 和完整四项 pin。
3. targets、Feeding status、MissionRequest 复制 task ID 和完整四项 pin。
4. FSM 保存本次 mission 预期地图身份。
5. identity 比较 current/task map，FSM 独立复核并在 mismatch 时关闭 Feeder。
6. Feeding 接单前 acquire lease，运行中 renew，终态 release。
7. activation 与 lease acquire 在 registry 同一锁内竞争。

完成条件：两张 version 相同、UUID 不同的地图不能互相冒充；UUID/version 相同但任一内容 hash 不同也必须拒绝。

### Phase 5：Web 列表、激活与状态展示

1. 接入 ROS services/status。
2. 实现三个 HTTP API。
3. 页面区分 working 和 active。
4. 显示 legacy 警告和失败码。
5. 运行中禁用激活按钮，但后端仍必须独立拒绝。

### Phase 6：新 Scout 产物 schema v2 与自动 promote

1. 新产物写入 reference、实际 environment identity、mapping config hash 和软件版本 hash。
2. 写完 manifest 后走同一个 activate service。
3. 自动激活失败保留旧 active，并在 Web/status 中明确显示。
4. 不创建第二套“内部快速激活”绕开完整验证。

### Phase 7：90-3 端到端验收

1. 部署并 `catkin_make`。
2. 干净启动 Gazebo/ROS/Web。
3. 激活历史任务样例。
4. 等待 60 秒。
5. preview 指定 cell。
6. start Feeding，验证身份门和投喂。
7. 完整重启后验证 selector 恢复。
8. 执行损坏、错 UUID、忙时换图等负向用例。

## 15. 必须通过的测试

### 15.1 单元测试

1. 完整成功产物恢复后 UUID、version、hash、坐标、协方差和身份不变。
2. 用真实 v1 字段矩阵验证样例；允许指定文件缺字段，但派生值与现有字段必须一致。
3. `allow_legacy_v1=false` 时拒绝 v1。
4. legacy reference、preset、seed、profile 或 environment ID/hash 不一致时拒绝。
5. v2 缺 reference/environment/mapping config/software hash 时拒绝。
6. manifest 登记的任意文件改一个字节，返回 `ARTIFACT_HASH_MISMATCH`。
7. 必需文件存在但未在 manifest 登记时拒绝。
8. manifest 含绝对路径、`../`、路径分隔符、symlink、非普通文件、重复 JSON key、NaN/Inf 或超大文件时拒绝。
9. map/result/quality/manifest 的字段按来源矩阵不一致时拒绝。
10. 非冻结、Scout 失败、mapping 失败、覆盖率不足时拒绝。
11. frame、anchor、reference、nominal geometry、environment identity 不一致时拒绝。
12. 重复/越界 cell、非法计数、错误 nominal/position_delta、非法协方差、非法 polygon 时拒绝。
13. 缺 edge、重复 edge、非法 edge 或非完整规范邻接集合时拒绝。
14. map 顶层与 metadata 重复字段冲突时拒绝。
15. 相同 UUID/version 但 map 或 manifest hash 不同，expected hash 检查必须拒绝。
16. 相同地图重复激活幂等，不创建新 version；同图 lease 存在时合同固定为幂等成功且不改变状态。
17. 候选在 commit point 前失败时旧 selector/active 保持。
18. selector 已 commit 后 publish/audit fault injection 进入 `REGISTRY_ERROR` 并可按 selector 恢复，不谎称旧图仍 active。
19. selector 写临时文件、fsync、replace、父目录 fsync 各故障点都有 fault-injection 测试。
20. 两个并发 activate 只有一个不同地图可以 commit。
21. activate 与 Feeding lease acquire 并发时由 registry 锁决定唯一顺序，不能出现运行中换图。
22. 两张 version 相同而 UUID 不同，下游返回 `TASK_MAP_MISMATCH`。
23. UUID/version 相同但 hash 不同，下游同样返回 `TASK_MAP_MISMATCH`。
24. 等待超过旧 30 秒门后仍能创建 Feeding preview。
25. registry heartbeat 超时后不能开始 Feeding；正在投喂时 heartbeat 超时，FSM 立即关闭 Feeder。
26. source map 真正超过策略寿命时拒绝，重发 topic 不能清零年龄。
27. source timestamp 在未来、恰好位于过期边界、系统墙钟回退时执行明确的保守策略。
28. `/clock` 暂停或回退时，heartbeat 仍按回调 monotonic 时间正确超时/恢复。
29. selector 的 map/manifest hash 与当前文件不一致时启动恢复失败，不自动选择最新任务。
30. list 遇到一个损坏目录时仍返回其他有效地图。
31. 新 Scout 失败后旧 active 四项 pin 不变。
32. 新消息字段 ROS 序列化、latched 恢复、map/status/hash 一致性通过。
33. 扩展消息 MD5 后，JSON artifact 恢复不依赖旧 bag；旧 bag 限制有明确测试或文档断言。

### 15.2 ROS 集成测试

1. 完全重启后激活 `scout-web-1791526104020`。
2. `/scout/lattice_map` 为 UUID `7ded...`、version 472、预期 map/manifest hash、60 个 confirmed cell。
3. Web 显示来源任务、墙钟完成时间、完整校验时间、hash 和 reference/environment legacy 警告。
4. 等待至少 60 秒后执行 SELECTED_CELLS preview/start。
5. `/feeding/target_cells`、Feeding status 和 MissionRequest 同时携带 task ID 与完整四项地图 pin。
6. Feeding lease 已 acquire/renew，Identity 与 FSM 保存的 task map 一致时允许投喂。
7. 模拟错误 identity/hash、registry 心跳中断或 lease 冲突后 FSM 立即关闭投喂。
8. 激活状态 selector 在 ROS/Gazebo/主机重启后仍可恢复。
9. clean output root 无 selector 时不加载任何旧图。
10. 启动新 Scout 时 active map 仍存在；只有成功产物可替换。
11. activate 与 Feeding start 并发压测中始终只有一个操作成功取得 registry 提交权。
12. ROS `/clock` 暂停/回拨时 wall heartbeat 仍工作，artifact source age 不被清零。

## 16. 90-3 验收命令模板

以下命令是实现完成后的验收目标，不代表当前代码已经支持这些 API：

```bash
cd ~/cage_uav_ws
catkin_make
source devel/setup.bash

roslaunch cage_simulation persistent_lattice_sim.launch \
  deformation_preset:=medium \
  deformation_seed:=42 \
  observation_source:=oracle_noisy \
  feeder_visual_source:=oracle_noisy \
  start_rviz:=true \
  start_web:=true \
  web_port:=8081
```

列出地图：

```bash
curl -s 'http://127.0.0.1:8081/api/scout/maps?cage_id=0&include_invalid=true' | jq
```

激活历史任务：

```bash
curl -s -X POST 'http://127.0.0.1:8081/api/scout/maps/activate' \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id":"scout-web-1791526104020",
    "cage_id":0,
    "expected_map_uuid":"7dedb190-3d6e-4d0d-b01c-5cb04ce549c0",
    "expected_map_version":472,
    "expected_map_sha256":"<从上一步列表复制>",
    "expected_manifest_sha256":"<从上一步列表复制>"
  }' | jq
```

检查 active 状态和地图：

```bash
rostopic echo -n 1 /scout/active_map_status
rostopic echo -n 1 /scout/lattice_map/map_uuid
rostopic echo -n 1 /scout/lattice_map/map_version
```

等待超过旧门限：

```bash
sleep 65
```

再执行 Feeding preview：

```bash
curl -s -X POST 'http://127.0.0.1:8081/api/preview' \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id":"feed-history-map-preview-001",
    "cage_id":0,
    "mode":"SELECTED_CELLS",
    "rows":[],
    "cols":[],
    "selected_cell_ids":["R03C05","R03C06"],
    "repeat_count":1,
    "feed_relative_alt":15.0,
    "transit_relative_alt":25.0
  }' | jq
```

检查 target 地图身份：

```bash
rostopic echo -n 1 /feeding/target_cells
```

验收时预期：

```text
map_uuid    = 7dedb190-3d6e-4d0d-b01c-5cb04ce549c0
map_version = 472
map_sha256/manifest_sha256 与 active status 和列表一致
targets     = R03C05, R03C06
不出现 SCOUT_MAP_STALE
```

## 17. 临时恢复方法的定位

在完整机制实现前，可以通过节点参数 `restore_map_file` 做一次开发验证，但它不是最终方案：

```text
/persistent_lattice/restore_map_file =
  /home/ubuntu/.ros/cage_uav/scout_runs/scout-web-1791526104020/map.json
```

它当前存在以下限制：

- 需要在节点启动前设置；
- 没有完整 manifest/result/quality 校验；
- 没有 runtime 切换和 Web 管理；
- working/active 没有隔离；
- 30 秒后 Feeding 和 identity 两处仍会判 stale；
- 只绑定 version，尚未全链路绑定 UUID 和内容 hash；
- legacy ENU reference 仍来自当前配置。

因此 Codex 不得把“launch 暴露一个 restore_map_file 参数”视为完成本计划。

## 18. Definition of Done

只有以下条件全部满足，才可以声明“历史 Scout 地图可长期供 Feeding 使用”已经完成：

- [ ] working map 与 active map 已隔离，且 `/scout/lattice_map` 只有 registry 一个发布者。
- [ ] 历史任务通过固定 output root 被发现，Web 不接受任意绝对路径。
- [ ] manifest 路径、symlink、文件类型、大小、重复 JSON key 和同 bytes hash/parse 均已安全校验。
- [ ] manifest 所有登记文件的 SHA-256 已重新验证。
- [ ] result、quality、map、manifest 跨文件字段已一致性验证。
- [ ] 只有 COMPLETED、mapping SUCCESS、frozen、coverage 合格的地图可激活。
- [ ] 地图结构、数值、reference 和 geometry 兼容性已验证。
- [ ] selector 原子持久化，重启后重新验证并恢复。
- [ ] 激活失败或新 Scout 失败不会覆盖旧 active map。
- [ ] Feeding 全任务和 Mission FSM 绑定 UUID/version/map hash/manifest hash。
- [ ] Feeding acquire/renew/release registry lease，activate 与接单并发竞态已关闭。
- [ ] Identity 与 FSM 对地图不一致 fail-closed。
- [ ] Feeding 运行中不能切图。
- [ ] 两个旧 30 秒判断都已替换为正确的 heartbeat/source-age 语义。
- [ ] heartbeat 用 wall publisher 和消费者 monotonic receipt age，不受 `/clock` 暂停/回拨影响。
- [ ] adapter 和 identity 使用 active map 内的 reference，不再各自静默读取不同运行参数。
- [ ] 激活后等待至少 60 秒仍可 preview/start。
- [ ] registry 停止心跳、artifact 损坏或 map identity 不一致时禁止投喂。
- [ ] `scout-web-1791526104020` 通过 legacy 仿真模式完成一次 Feeding preview 和完整任务。
- [ ] ROS/Gazebo 完全重启后仍能恢复同一 UUID/version/map hash/manifest hash。
- [ ] 单元测试、ROS 集成测试和 90-3 在线验收记录全部通过。

## 19. 给后续 Codex 的严格执行规则

1. 先按 Phase 0 补测试，再按 Phase 1 到 Phase 7 顺序实现。
2. 每完成一个 Phase 就运行该层测试，不能攒到最后一起验证。
3. 优先复用 `PersistentLatticeMap.load_json()`、原子写文件和现有 target 生成逻辑。
4. 不允许用“把 30 秒改成很大”或“周期刷新 header”作为修复。
5. 不允许用 version 代替完整 UUID/version/map hash/manifest hash pin。
6. 不允许让 Web 传绝对文件路径。
7. 不允许只靠 topic 状态检查来防止 Feeding 活跃时切图，必须使用 registry lease。
8. 不允许新 Scout reset 直接销毁旧 active map。
9. 不允许为了让旧样例通过而关闭真实飞行的 reference/geometry 校验。
10. 不允许未经 60 秒等待、进程重启和负向损坏测试就宣称长期复用完成。
11. 实现后更新 `SCOUT_WEB_MISSION_IMPLEMENTATION_REPORT.md`，记录实际文件、测试命令、90-3 运行结果和仍存在的限制。
