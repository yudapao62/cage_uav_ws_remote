# Scout-Feeder 不规则持久格网仿真实施计划

> 状态：实施前设计稿  
> 日期：2026-10-08  
> 目标工程：`/Users/yudapao/cage_uav_ws_remote`  
> 文档依据：`Scout_Feeder_视觉方法设计.pdf`、现有无人机投喂仿真视频、当前 ROS/Gazebo 工程快照

## 0. 一句话方案

把当前“严格规则的 6 x 10 网格 + 由公式直接算中心点”改造成两层结构：

1. **拓扑层保持规则且身份稳定**：每个 cell 永久使用 `(row, col)`，相邻关系不因漏检、出视野或几何变形而改变。
2. **几何层允许不规则并持续修正**：网箱由共享顶点形成连续、轻微弯曲和不等距的四边形格网；Scout 在线维护每格的估计坐标与不确定性，Feeder 在 GPS 缩小的相邻候选中恢复当前 cell，并在证据不足时输出 `AMBIGUOUS`，不强行投喂。

核心原则是：**规则拓扑 + 非规则几何 + 持久身份 + 可拒判的不确定性**。

---

## 1. 参考材料与当前判断

### 1.1 PDF 中必须落地的主线

PDF 的方法不是普通目标检测或逐帧编号，而是：

- Scout 从连续但不完整的局部观测中维护持久格网 `M_t = (V_t, E_t)`。
- 每个 cell 的身份、位置和状态分开保存。
- 局部格网 `(i, j)` 通过整数偏移 `Delta_t = (Delta_r, Delta_c)` 对应到全局 `(r, c)`。
- 暂时漏检不能触发后续 cell 整体错一格。
- 新 cell 只从持久格网的开放边界出生，并经过多帧确认。
- 名义位置与实际位置分开：

  ```text
  p_nominal(r,c) = p_anchor + r * b_row + c * b_col
  p_actual(r,c)  = p_nominal(r,c) + delta(r,c)
  ```

- Feeder 用 GPS 生成少量候选，再结合局部邻域视觉和任务历史计算后验。
- Top-1 与 Top-2 接近时保留多假设，禁止为了“有答案”而强制选择一个相邻 cell。

### 1.2 视频能提供什么

当前找到的对应视频为：

```text
/Users/yudapao/Library/Containers/com.tencent.xinWeChat/Data/Documents/
xwechat_files/wxid_mnhd297ai2yn12_0747/msg/video/2026-10/
dd9e14443e9752cd8e2a3eca9b9b34a1.mp4
```

视频约 2 分 29 秒、960 x 544，展示了现有 Gazebo + RViz + Web 投喂 demo：

- 约 6 x 10 的规则矩形网格；
- `R01C01 ... R06C10` 编号；
- `ROW_OR_COLUMN` 和 `FULL_GRID` 路线；
- 当前/已完成 cell 和规划轨迹可视化。

这个视频适合作为“现有功能基线”和 UI/任务链参考，但视频中的仿真网箱仍然规则，并没有展示目标中的明显非规则形变。非规则结构第一版以 PDF 第 3 页真实网箱照片中的轻微弯曲、不等距、局部拉伸和遮挡为依据。如果用户所指的是另一个尚未落盘的视频，后续只需要重新标定形变参数，不需要推翻本计划的架构。

### 1.3 当前工程快照不是最终远端真值

本地目录是 2026-09-21 从远端工程拉取的源码快照，没有 `.git` 元数据。正式编码前应先对比目标主机当前 `/home/ubuntu/cage_uav_ws/src`，避免在旧快照上实现后覆盖远端新改动。

本地也没有 `VLX-Seek`、WeDetect、CageGrid 视觉源码或完整视觉 v9 checkpoint。现有 v9/SCOPE 文件属于另一条动态占据预测链，不能视为 Scout/Feeder 网箱视觉已经实现。

---

## 2. 现有系统盘点：保留什么，缺什么

### 2.1 已有能力

| 能力 | 当前实现 | 本项目处理方式 |
|---|---|---|
| 网箱场景生成 | `cage_simulation/scripts/generate_fish_farm.py` | 扩展为共享形变顶点生成器，不重写整套场景 |
| 场景参数 | `cage_simulation/config/fish_farm.yaml` | 增加结构形变、缺口、种子和 GT 输出配置 |
| UAV 相机 | `generate_uav_camera.py`、`camera.yaml`、`/uav_camera/image_raw` | 直接复用；增加视觉节点订阅者 |
| 水面、梁、浮筒、网衣 | 已有程序化生成 | 保留材质与外观随机性，但与结构真值分层 |
| 规则 cell 模型 | `cage_grid/src/cage_grid/model.py` | 保留 cell ID、任务顺序和兼容层；新增不规则格网模型 |
| cell 任务模式 | `FULL_GRID`、`ROW_OR_COLUMN`、`SELECTED_CELLS` | 全部保留，目标坐标改读 Scout 地图快照 |
| 航点/FSM/MAVROS | planner、mission FSM、adapter 已存在 | 尽量不改状态机，只改目标来源和安全门控 |
| 投料器、Web、RViz | 已存在 | 扩展显示估计格网、置信度、候选和歧义状态 |
| 测试基础 | geodesy/grid/planner/FSM/visualization 单测 | 增加算法单测、回放测试和场景矩阵 |

### 2.2 现有“不规则”为什么不够

`generate_fish_farm.py` 当前对每个网格顶点施加约 `+-0.08 m` 的独立微扰，并对梁角、网衣和浮筒做外观扰动。相对于 12 m 单格，这主要是消除 CAD 感：

- 不形成连续的整体弯曲；
- 不形成缓慢变化的行/列间距；
- 不表达局部拉伸、剪切或结构缺口；
- Gazebo 外观扰动没有同步给任务几何；
- `CageGridModel.cell_center()` 仍按统一 `cell_size_x/y + yaw` 计算严格规则中心。

因此当前 Gazebo 看到的外观和任务系统使用的几何不是同一份真值，这一点必须先修正。

### 2.3 明确缺口

当前工程没有：

- Scout 图像前端；
- 局部格网观测消息；
- 跨帧整数偏移关联；
- 持久 cell ID 和 birth/visibility 状态机；
- 每格修正坐标与协方差；
- 漏检恢复、多假设和冻结高风险写入；
- Feeder GPS 候选、邻域匹配、任务后验和拒判；
- 图像/GT/关联/任务状态的统一记录与自动评测。

### 2.4 必须保留的兼容性

当前 `/cage/geometry_mode` 已用于区分 `rectangular` 和 `circular` 两套任务。新的不规则网箱仍然是 row-column 矩形拓扑，因此：

- **不要新增第三个 `geometry_mode=irregular`**；
- 保持 `geometry_mode=rectangular`；
- 新增独立参数，例如 `lattice_geometry_mode: nominal | deformed`；
- 新建专用 launch/config，不改坏现有 circular center-feeding 默认配置。

---

## 3. 目标范围和非目标

### 3.1 本轮必须实现

1. 可复现的不规则矩形网箱 Gazebo 场景。
2. 与场景完全一致的 cell 顶点、中心、邻接和锚点 GT。
3. 可在 `oracle_noisy` 和 `vision` 两种输入源间切换的 Scout 接口。
4. Scout 持久 row-column 身份、漏检恢复、边界 birth、多假设和坐标修正。
5. Feeder 的 GPS 候选、邻域证据、任务历史后验和 `AMBIGUOUS` 拒判。
6. 把确认后的 cell 坐标接入现有任务、航点、RViz 和 Web 链。
7. 可回放、可统计、可按场景强度分层的自动测试。

### 3.2 第一版明确不做

- 不做海洋 CFD 或结构受力仿真。
- 不把网箱做成每帧大幅摆动的软体。
- 不在第一阶段训练端到端 Scout-Feeder 大模型。
- 不允许算法读取未来帧。
- 不让算法节点订阅 GT topic，除非运行在明确的 `oracle_noisy` 验证模式。
- 不用 GPS 最近邻直接替代 Feeder 身份恢复。
- 不因漏检删除 confirmed cell。
- 不在身份尚未确定时用该观测更新全局坐标。

### 3.3 分阶段处理拓扑异常

第一版“非规则”指几何非规则，默认仍有完整二维 row-column 拓扑：

```text
P0: 完整拓扑 + 不规则几何
P1: 完整拓扑 + 视觉遮挡/漏检/未闭合边
P2: 明确的物理缺口或维护区域
P3: 只有真实数据确认存在分叉/合并时，才升级为一般图匹配
```

这样可以先验证 PDF 中最核心的整数偏移与身份保持问题，避免一开始把“物理不存在”和“暂时看不见”混为一谈。

---

## 4. 总体架构

```text
fish_farm.yaml + random_seed
            |
            v
  Shared Deformed Vertex Grid
       |                 |
       |                 +--> Gazebo SDF / meshes / visuals
       |
       +--> /sim_gt/cage/lattice  (仅评测与 oracle 适配器可见)
                             |
UAV camera + pose + GPS      |
       |                     |
       v                     v
Local Lattice Frontend <--- Oracle Noisy Adapter
       |
       v
Scout Integer-Offset Association
       |
       +--> Birth / Visibility / Multi-hypothesis
       |
       +--> Coordinate Optimizer
       |
       v
/scout/lattice_map (identity + estimated position + covariance + version)
       |
       +-------------------------+
       |                         |
       v                         v
Existing Task Adapter       Feeder Candidate Generator
/feeding/target_cells       GPS + Scout covariance
       |                         |
       v                         v
Existing Planner/FSM        Local neighborhood matcher
                                 |
                                 v
                         Task-state Bayesian filter
                                 |
                                 v
                   CONFIRMED / AMBIGUOUS / LOST
```

实施顺序必须先打通 `GT -> oracle_noisy -> Scout/Feeder 后端`，再接真实图像前端。这样能区分“身份逻辑错了”与“视觉检测不准”，不会把所有问题都归到模型训练上。

---

## 5. 坐标、索引和真值约定

这些约定必须在写代码前固定，否则 row/column 翻转和 180 度歧义会长期污染数据。

### 5.1 坐标系

- ROS 地图与算法统一使用 ENU `map`。
- Gazebo 仍遵循当前工程转换：world `x = North`、`y = -East`、`z = Up`。
- 只有场景生成器负责 ENU 到 Gazebo world 的转换，算法侧禁止重复转换。
- 相机观测保留原始 `camera optical frame`，通过时间同步 TF/位姿投影到 `map`。

### 5.2 索引

- 顶点索引：`vertex[i][j]`，从 0 开始，尺寸为 `(rows + 1) x (cols + 1)`。
- cell 索引：`row=1..rows`、`col=1..cols`。
- `R01C01` 由 `vertex[0][0]、vertex[0][1]、vertex[1][1]、vertex[1][0]` 围成。
- `col` 沿局部 `+x` 方向增长，`row` 沿局部 `+y` 方向增长，再由全局 yaw 旋转到 ENU。
- 锚点 `p_A` 固定定义为 `vertex[0][0]`，不是 cell 中心。

### 5.3 三套几何必须分开

| 几何 | 含义 | 谁可读取 |
|---|---|---|
| `nominal` | 锚点 + 名义行列步长推算的规则格网 | 所有算法，可作为弱先验 |
| `ground_truth` | 仿真生成器实际使用的变形顶点/中心 | 仅仿真、oracle 和评测 |
| `estimated` | Scout 在线估计的 cell 坐标及协方差 | Feeder、任务和可视化 |

严禁把 GT 直接发布到生产 topic，也严禁用 GT 覆盖 Scout 估计后再声称完成了坐标修正。

---

## 6. 非规则网箱生成

### 6.1 正确的数据结构

不单独扰动 60 个 cell 中心，而是先生成 7 x 11 个共享顶点，再由相邻四个顶点构成 cell。这样能保证：

- 相邻 cell 真正共边；
- 不会出现独立中心抖动造成的重叠和裂缝；
- 梁、浮筒、网衣、GT polygon 和 cell 中心来自同一几何；
- row-column 邻接关系天然保留。

### 6.2 形变组成

对名义顶点 `q_nominal(i,j)` 依次施加：

1. 全局 yaw 和轻微 shear；
2. 行间距与列间距的低频缓慢变化；
3. 沿整行/整列连续变化的 bend；
4. 相关长度大于 1 个 cell 的二维平滑形变场；
5. 很小的局部残差扰动；
6. 可选的结构缺口标记，但不破坏未标记 cell 的共享顶点。

形变应“连续且低频”，不能对每个点独立采样大幅高斯噪声。

### 6.3 推荐配置

在 `fish_farm.yaml` 中增加：

```yaml
fish_farm:
  lattice_geometry_mode: deformed

  deformation:
    enabled: true
    random_seed: 42

    # 初始调参范围，最终按真实视频标定
    row_spacing_cv: 0.04
    col_spacing_cv: 0.04
    smooth_warp_amplitude_m: 0.8
    warp_correlation_cells: 3.0
    bend_amplitude_m: 0.6
    shear_deg: 2.0
    local_residual_std_m: 0.08

    validation:
      min_edge_length_ratio: 0.70
      max_edge_length_ratio: 1.30
      min_cell_area_ratio: 0.55
      min_internal_angle_deg: 35.0

  structural_holes: []
  visual_occluders: []
```

这些数值只是 12 m cell 下的仿真起点，不代表现场测量值。后续应建立 `mild / medium / strong` 三个 preset，并保存对应 seed。

### 6.4 生成器输出

同一次生成必须同时产出：

```text
cage_simulation/models/fish_farm/model.sdf
cage_simulation/models/fish_farm/meshes/*.obj
cage_simulation/generated/fish_farm_gt.json
cage_simulation/generated/fish_farm_preview.png   # 可选离线检查
```

`fish_farm_gt.json` 至少包含：

```json
{
  "schema_version": 1,
  "seed": 42,
  "frame_id": "map",
  "anchor_definition": "vertex_0_0",
  "anchor_enu": [0.0, 0.0, 0.0],
  "rows": 6,
  "cols": 10,
  "vertices": [],
  "cells": [
    {
      "row": 1,
      "col": 1,
      "cell_id": "R01C01",
      "vertex_indices": [[0,0],[0,1],[1,1],[1,0]],
      "center_enu": [0.0, 0.0, 0.0],
      "neighbors": ["R01C02", "R02C01"],
      "exists": true
    }
  ]
}
```

### 6.5 几何生成时的硬校验

生成失败而不是静默接受以下情况：

- cell polygon 自交；
- 有向面积为负或接近 0；
- row/column 顺序翻转；
- 边长、面积或内角超出配置阈值；
- 相邻 cell 没有共享完全相同的边顶点；
- JSON 与 SDF 使用不同 seed 或不同坐标变换；
- 相同 seed 不能复现完全相同的 GT 哈希。

### 6.6 多网箱

`multi_cage_layout.yaml` 当前没有真正驱动 world 生成，world 中 5 个实例姿态是硬编码的。应让 world 生成器消费该 YAML，并允许每个网箱有独立 `deformation_seed`。不能把一个变形模型刚体复制 5 次，否则远处所有网箱的局部缺陷完全相同，会产生不真实的视觉捷径。

---

## 7. ROS 消息与 topic 设计

### 7.1 兼容策略

- 保留现有 `CageGrid.msg` 和 `/cage/grid_info`，作为名义规则格网摘要与旧节点兼容接口。
- 新增 `PersistentLattice.msg`，表达每格实际估计、状态、协方差和地图版本。
- 现有 planner 仍消费 `CellTargetArray`；由任务适配器从一个稳定的 Scout map snapshot 生成实际坐标。
- 不直接扩展旧消息后假设所有旧节点都能理解新语义。

### 7.2 建议新增消息

```text
cage_msgs/msg/LatticeCell.msg
cage_msgs/msg/LatticeEdge.msg
cage_msgs/msg/PersistentLattice.msg
cage_msgs/msg/LocalLatticeCell.msg
cage_msgs/msg/LocalLatticeObservation.msg
cage_msgs/msg/OffsetHypothesis.msg
cage_msgs/msg/OffsetHypothesisArray.msg
cage_msgs/msg/FeederCellHypothesis.msg
cage_msgs/msg/FeederIdentityState.msg
```

`LatticeCell.msg` 最少字段：

```text
int32 row
int32 col
string cell_id
geometry_msgs/Point nominal_position
geometry_msgs/Point estimated_position
float64[4] covariance_xy
uint8 identity_state       # TENTATIVE / CONFIRMED / REJECTED
uint8 visibility_state     # VISIBLE / OUT_OF_VIEW / EXPECTED_BUT_UNOBSERVED
float32 identity_confidence
time first_seen
time last_seen
uint32 observation_count
```

`PersistentLattice.msg` 最少字段：

```text
std_msgs/Header header
int32 cage_id
uint64 map_version
string map_uuid
string anchor_definition
geometry_msgs/Point anchor_enu
cage_msgs/LatticeCell[] cells
cage_msgs/LatticeEdge[] edges
```

### 7.3 建议 topic

| Topic | 发布者 | 订阅者 | 说明 |
|---|---|---|---|
| `/sim_gt/cage/lattice` | GT publisher | evaluator/oracle only | 隐藏真值 |
| `/scout/local_lattice_observation` | vision 或 oracle adapter | Scout association | 当前帧局部 `(i,j)` |
| `/scout/offset_hypotheses` | Scout association | debug/evaluator | Top-K `Delta_r,Delta_c,p` |
| `/scout/lattice_map` | persistent map node | Feeder/task/RViz/Web | 持久身份和修正坐标 |
| `/scout/map_events` | persistent map node | logger | birth/confirm/miss/recover/freeze |
| `/feeder/cell_hypotheses` | Feeder filter | debug/evaluator | 候选分项 likelihood 与后验 |
| `/feeder/identity_state` | Feeder filter | feeder gate/FSM/Web | previous/current/next/ambiguous |
| `/feeding/target_cells` | existing task manager/adapter | planner/grid markers | 保持现有接口 |

### 7.4 地图版本与任务冻结

- Scout 每次结构提交后递增 `map_version`。
- 投喂任务开始时记录 `task_map_version` 并冻结该版本的 target 坐标。
- Scout 可继续后台更新，但第一版不在任务执行中静默移动已上传航点。
- 若坐标变化超过安全阈值，输出 `REPLAN_REQUIRED`，由任务层显式取消/重规划。
- Feeder 查询可以使用较新地图，但必须在日志中同时记录查询版本和任务版本。

---

## 8. Scout 实现

### 8.1 两种观测源

统一参数：

```text
observation_source := oracle_noisy | vision_v9
```

`oracle_noisy`：

- 只取当前时刻、当前 FoV 内的 GT cell；
- 加入像素/地面坐标噪声；
- 随机或脚本化漏检；
- 模拟未闭合边、遮挡、反光导致的置信度下降；
- 不提供全局 `(r,c)` 给 Scout，只输出局部 `(i,j)` 和观测几何。

`vision_v9`：

- 订阅 `/uav_camera/image_raw` 与 `/uav_camera/camera_info`；
- 输出交点、两个主方向、cell quad、边界证据、局部索引和置信度；
- 不直接预测永久全局编号；
- 通过适配器接入现有/后续 CageGrid、VP-lattice 或 v9 前端。

### 8.2 局部格网前端

前端只回答当前画面中的结构：

- 两个透视线族/主方向；
- 局部交点和相邻关系；
- 完整或部分 cell；
- 每条边是 `observed` 还是 `inferred`；
- 观测置信度和有效 FoV 边界；
- 局部 `(i,j)`，方向归一化后再输出。

必须显式处理轴交换和 180 度翻转。方向归一化使用 UAV 航向、相机姿态和历史格网，不能仅靠图像左上角排序。

### 8.3 整数偏移 Top-K 关联

对于当前局部格网与历史地图，枚举有限的整数偏移候选：

```text
Delta_t = (Delta_r, Delta_c)
r = i + Delta_r
c = j + Delta_c
```

每个候选用概率因子计算 log posterior：

- 当前局部视觉与历史 cell prototype/几何是否相容；
- row-column 邻接、顺序和缺口解释是否一致；
- 地面投影是否落在历史坐标协方差允许范围；
- 相邻帧运动模型对 `Delta_t` 的预测；
- 候选是否需要不合理地删除 confirmed cell 或在内部生成新 cell。

保留 Top-K，建议第一版 `K=3`。只有满足以下条件才提交唯一偏移：

- Top-1 posterior 达到校准阈值；
- Top-1 与 Top-2 的 margin/odds 达到阈值；
- 没有轴交换/180 度对称歧义；
- 提交不会造成内部非法 birth 或大范围身份改写。

否则：

- 发布多假设；
- 更新可见性但暂停高风险结构写入；
- 等待下一帧证据，不能强行选择。

### 8.4 cell 状态机

身份状态与可见性分别维护。

身份：

```text
TENTATIVE -> CONFIRMED
TENTATIVE -> REJECTED
CONFIRMED 不因单帧漏检回退或删除
```

可见性：

```text
VISIBLE
OUT_OF_VIEW
EXPECTED_BUT_UNOBSERVED
```

`EXPECTED_BUT_UNOBSERVED` 很重要：它表示 cell 根据相机位姿本应在 FoV 内，但视觉没有检测到；这才是“漏检”，与正常离开视野不同。

建议第一版确认逻辑：

- 新 cell 必须位于当前已确认格网开放边界外；
- 连续或近邻窗口内至少 3 次一致观测；
- 坐标、邻接和身份偏移都通过门控；
- 内部空洞默认解释为临时漏检，不允许直接 birth 出一个新 ID；
- 多假设未收敛时不做 birth。

具体帧数后续通过验证集标定，不在模型内部写死。

### 8.5 坐标修正

只有身份关联提交后才产生坐标因子。优化变量是 confirmed cell 的 ENU 位置，因子包括：

1. 唯一真实锚点的强约束；
2. 名义规则格网的弱先验；
3. 相邻 cell 的视觉相对位移；
4. 多帧相同 cell 的重复观测；
5. 可选的平滑形变先验；
6. Huber/Cauchy 鲁棒核，限制异常匹配污染。

第一版可用稀疏非线性最小二乘定期增量求解，不必一开始引入复杂 SLAM 框架。每次优化输出：

- `estimated_position`；
- `covariance_xy`；
- 观测数与最后更新时间；
- 相对名义位置的 `delta(r,c)`；
- 优化残差和被鲁棒核抑制的异常因子数。

坐标协方差随长时间未观测而膨胀；不能让过期地图保持虚假的高置信度。

### 8.6 持久化和复位

- 支持保存/加载版本化 JSON 或 rosbag 快照。
- 保存 `map_uuid`、配置哈希、seed、锚点定义和消息 schema 版本。
- 仿真每次 clean run 默认新建地图。
- 继续任务必须显式指定恢复的地图，禁止自动拾取未知旧文件。

---

## 9. Feeder 实现

### 9.1 GPS 只负责候选生成

对每个 confirmed cell 计算：

```text
innovation = feeder_gps_xy - scout_cell_xy
S = GPS_covariance + cell_covariance + map_staleness_covariance
mahalanobis_d2 = innovation.T * inv(S) * innovation
```

按卡方门控生成候选，而不是固定欧氏半径或最近邻。输出中保留：

- 候选 `(r,c)`；
- GPS likelihood；
- cell 坐标协方差；
- 地图年龄；
- 候选进入门控的原因。

若地图不确定性变大，候选集合应自然扩大。

### 9.2 比较局部邻域而不是单格外观

对每个 GPS 候选读取以该 cell 为中心的局部子图，例如 3 x 3 或沿任务方向的 1 x 3：

- 中心 cell 结构；
- 左右/上下邻接顺序；
- 相对中心的位置和尺度；
- 边界、缺口、网衣/浮筒等可用局部特征；
- 当前小 FoV 中可见节点与边的匹配比例。

第一版先用 oracle/noisy 几何构造 `L_visual`，确认 Bayesian filter 正确；再接 Feeder 低空图像前端。

### 9.3 任务状态后验

后验包含：

```text
posterior = visual_likelihood
          * gps_likelihood
          * transition_prior(previous_cell, route_mode, reentry_state)
```

转移模型：

- 连续投喂：高概率转移到规划路线的下一格；
- 悬停重观测：保留当前格较高概率；
- 返航重入：主动放宽，不强制从上一个 cell 延续；
- 手动接管/任务中断：清除过强连续性，保留任务进度但扩大候选；
- 方向反转：从任务计划读取，不由图像猜测。

### 9.4 提交与拒判

输出状态：

```text
CONFIRMED    # 可以继续当前 cell 的任务
AMBIGUOUS    # 保留多候选，禁止触发投喂
LOST         # 真值很可能不在当前候选或证据完全不足
STALE_MAP    # Scout 地图过期/协方差过大
```

`CONFIRMED` 至少需要：

- Top-1 posterior 达阈值；
- Top-1 与 Top-2 margin 达阈值；
- 总熵低于阈值；
- 当前 map version、GPS 时间和图像时间满足同步要求；
- 候选 cell 为 confirmed 且坐标不过期。

阈值通过验证集做 calibration，不能从单个 demo 手调后写死。

### 9.5 接入现有任务链

- `FULL_GRID` 继续按 row-column 蛇形顺序生成任务，但每个 waypoint 使用 Scout 修正中心。
- `ROW_OR_COLUMN` 继续按稳定身份筛选，不按空间临时排序改变 cell ID。
- `SELECTED_CELLS` 保持现有 Web 选择方式。
- planner/FSM 不直接订阅视觉候选；只接收任务适配器批准的稳定 target。
- Feeder 返回 `AMBIGUOUS/LOST/STALE_MAP` 时，投料器 gate 必须关闭，FSM 进入等待、重观测或安全退出，而不是继续执行下一格。

---

## 10. 视觉前端与已有 v9/几何工作的结合

### 10.1 接口优先，不等待模型

当前本地 ROS 工程没有视觉代码，设计文档中提到的 VP-lattice/CageGrid/VLX 工作也未集成。因此先固定 `LocalLatticeObservation` 接口，让任一前端都能接入：

```text
image
  -> line/intersection/cell proposal
  -> direction normalization
  -> local lattice (i,j)
  -> observed/inferred edges + confidence
  -> LocalLatticeObservation
```

### 10.2 可复用内容

若远端已有 v9 或 CageGrid 结果，优先复用：

- VP 约束的两组线；
- 交点与 cell quad；
- observed/inferred 边；
- line/cell confidence；
- 局部 row/column 排序。

不应直接复用或相信：

- 每帧从 1 开始的全局编号；
- 仅按 bbox 左上角排序得到的 row/col；
- 纯水面外推出来的 cell；
- 没有时间戳和相机位姿的离线 JSON；
- 用未来帧修正当前输出的离线结果。

### 10.3 训练顺序

1. 先冻结已有视觉前端，验证后端身份逻辑。
2. 用仿真自动生成局部格网 GT、遮挡和形变分层数据。
3. 训练/微调局部交点、边界和置信度，不直接端到端预测全局 `(r,c)`。
4. 加入相邻帧正样本、`+-1 row/col slip` 困难负样本和边界 birth 样本。
5. Feeder 使用同排/同列相邻 cell 作为核心困难负样本，随机远处负样本只作补充。
6. 单独做不确定性 calibration，检查 NLL/ECE 和 risk-coverage。
7. 最后再评估 synthetic-to-real 域差，必要时使用真实视频微调。

---

## 11. 仿真故障注入与场景矩阵

### 11.1 结构形变

```text
none / mild / medium / strong
```

分别改变 bend、spacing CV、shear、smooth warp，但始终记录 seed 和几何有效性。

### 11.2 Scout 观测故障

- 单 cell 漏 1 帧；
- 同一 cell 连续漏 3/5 帧；
- 连续一段 row/column 局部不可见；
- FoV 边缘只出现部分 cell；
- 开放边界新 cell 逐渐进入；
- 反光导致边置信度下降；
- 姿态/高度误差导致地面投影偏移；
- 局部轴交换或方向反转候选；
- 两个整数偏移 posterior 接近。

### 11.3 Feeder 故障

- GPS 白噪声；
- 时间相关 bias/random walk；
- GPS 丢包与延迟；
- Scout map 坐标不确定性变大；
- 低空小 FoV 仅见 1/2/3 个 cell；
- 连续投喂与返航重入；
- 同排三选一 `{(r,c-1),(r,c),(r,c+1)}`；
- 地图过期与任务 map version 不一致。

### 11.4 防止测试泄漏

- 训练、调参和最终评测使用不同 seed 集合。
- 形变 preset 与随机 seed 都写入结果。
- 算法进程的 ROS namespace 中不提供 `/sim_gt/*`。
- evaluator 只做离线比对，不回写算法 topic。

---

## 12. 评测指标与验收门槛

### 12.1 Scout 指标

| 指标 | 定义 |
|---|---|
| Row/column identity accuracy | 已提交关联中 `(r,c)` 正确比例 |
| Identity switch rate | 同一 GT cell 被改成其他永久身份的次数/轨迹长度 |
| Slip rate | 发生整行/整列 `+-1` 偏移的事件率 |
| Birth precision/recall | 新 cell 是否只在正确开放边界、正确格位出生 |
| Birth confirmation delay | 从首次可见到 confirmed 的帧数 |
| Missed-cell recovery rate | 漏检后恢复原 ID、没有新建 ID 的比例 |
| Coordinate RMSE | Scout 估计中心相对 GT 的误差 |
| Nominal improvement | 修正坐标相对名义规则坐标的改善比例 |
| Coverage / abstention | 提交比例与错误率的权衡 |

### 12.2 Feeder 指标

| 指标 | 定义 |
|---|---|
| Candidate recall | 真值 cell 是否进入 GPS 候选集合 |
| Top-1 / Top-k accuracy | 候选后验排序正确率 |
| Adjacent-cell error rate | 错误提交到上下左右邻格的比例 |
| Wrong-commit rate | 系统提交唯一 ID 但提交错误的比例 |
| Ambiguous recall | 真正歧义样本是否被拒判 |
| Re-entry recovery time | 返航重入后恢复稳定身份的时间/帧数 |
| NLL / ECE | 后验概率是否校准 |
| Feeding success | 最终在正确 cell 允许投喂的任务比例 |

### 12.3 第一轮工程门槛

以下是第一轮验收目标，完成基线测量后可以通过评审调整，但不能无记录降低：

1. 无噪声 oracle 场景：identity accuracy 100%，slip 0，wrong commit 0。
2. 单 cell 漏 1-3 帧：不得造成后续整排/整列 slip；恢复时必须回到原 ID。
3. confirmed cell 离开 FoV：地图仍保留，重新进入时不创建新 ID。
4. 中等形变 oracle-noisy 场景：identity accuracy >= 99.5%，slip 事件为 0。
5. 坐标修正：RMSE 必须显著优于 nominal；初始目标为改善 >= 30%。
6. Feeder GPS candidate recall >= 99%。
7. 相邻三选一测试：wrong-commit <= 1%；无法满足时应增加 abstention，而不是放宽提交。
8. 所有 `AMBIGUOUS/LOST/STALE_MAP` 状态下，投料器触发次数必须为 0。
9. 现有 rectangular/circular demo、任务模式和手动安全链必须回归通过。

### 12.4 必须对比的基线

- 每帧从左到右/从上到下重新编号；
- 始终使用名义规则格网；
- 普通 track ID，不使用 row-column 拓扑；
- Feeder 仅用 GPS 最近 cell；
- Feeder 仅用单格外观 Top-1；
- 完整方法去掉 task transition prior；
- 完整方法去掉坐标协方差；
- 完整方法不允许 abstain。

---

## 13. 记录、回放和可视化

### 13.1 rosbag topic

至少记录：

```text
/clock
/tf
/tf_static
/uav_camera/image_raw
/uav_camera/camera_info
/mavros/local_position/pose
/mavros/global_position/global
/sim_gt/cage/lattice
/scout/local_lattice_observation
/scout/offset_hypotheses
/scout/lattice_map
/scout/map_events
/feeder/cell_hypotheses
/feeder/identity_state
/feeding/target_cells
/feeding/task_status
/mission/state
```

### 13.2 事件日志

额外输出 JSONL，每条记录包含：

- 时间戳与 frame sequence；
- scenario、seed、配置哈希；
- GT/nominal/estimated map version；
- Top-K 偏移或 cell 候选；
- 每个 likelihood 分项；
- commit/abstain 原因；
- birth/miss/recovery/freeze/replan 事件；
- 当前任务 cell、previous/current/next。

### 13.3 RViz

用不同视觉编码同时显示：

- 名义规则格网：细灰线；
- Scout 修正格网：彩色实线；
- covariance：椭圆；
- tentative：黄色虚线；
- confirmed visible：绿色；
- confirmed out-of-view：蓝色；
- expected-but-unobserved：橙色；
- rejected：红色；
- Feeder 候选：透明度表示 posterior；
- ambiguous：显示 Top-2，不高亮唯一当前格。

### 13.4 Web

保留当前格子选择与任务控制，在状态区新增：

- `map_version`、`map_age`；
- Scout committed offset 与置信度；
- Feeder Top-3 候选；
- `CONFIRMED/AMBIGUOUS/LOST/STALE_MAP`；
- 如果任务冻结旧 map，显示 `task_map_version != latest_map_version`。

---

## 14. 建议文件改动

### 14.1 扩展现有文件

```text
src/cage_simulation/config/fish_farm.yaml
src/cage_simulation/scripts/generate_fish_farm.py
src/cage_simulation/scripts/start_full_simulation.sh
src/cage_grid/src/cage_grid/model.py
src/cage_grid/scripts/cage_grid_node.py
src/cage_feeding_task/scripts/feeding_task_manager_node.py
src/cage_visualization/scripts/cage_marker_node.py
src/cage_web_bridge/scripts/web_bridge_node.py
src/cage_web_bridge/www/index.html
src/cage_msgs/CMakeLists.txt
```

### 14.2 新增核心文件

```text
src/cage_simulation/scripts/generate_multi_cage_world.py
src/cage_simulation/scripts/cage_ground_truth_publisher.py
src/cage_simulation/scripts/synthetic_lattice_observation_node.py
src/cage_simulation/config/persistent_lattice_sim.yaml
src/cage_simulation/launch/persistent_lattice_sim.launch

src/cage_grid/src/cage_grid/deformed_model.py
src/cage_grid/src/cage_grid/persistent_map.py
src/cage_grid/src/cage_grid/association.py
src/cage_grid/src/cage_grid/coordinate_optimizer.py
src/cage_grid/src/cage_grid/feeder_filter.py
src/cage_grid/scripts/scout_lattice_node.py
src/cage_grid/scripts/feeder_identity_node.py
src/cage_grid/config/persistent_lattice.yaml

src/cage_msgs/msg/LatticeCell.msg
src/cage_msgs/msg/LatticeEdge.msg
src/cage_msgs/msg/PersistentLattice.msg
src/cage_msgs/msg/LocalLatticeCell.msg
src/cage_msgs/msg/LocalLatticeObservation.msg
src/cage_msgs/msg/OffsetHypothesis.msg
src/cage_msgs/msg/OffsetHypothesisArray.msg
src/cage_msgs/msg/FeederCellHypothesis.msg
src/cage_msgs/msg/FeederIdentityState.msg

src/cage_bringup/scripts/persistent_lattice_acceptance.py
src/cage_bringup/scripts/evaluate_persistent_lattice.py
```

### 14.3 新增测试

```text
src/cage_simulation/test/test_deformed_farm_generator.py
src/cage_grid/test/test_persistent_map.py
src/cage_grid/test/test_integer_offset_association.py
src/cage_grid/test/test_birth_visibility.py
src/cage_grid/test/test_coordinate_optimizer.py
src/cage_grid/test/test_feeder_candidate_gate.py
src/cage_grid/test/test_feeder_filter.py
src/cage_feeding_task/test/test_persistent_map_adapter.py
```

---

## 15. 分阶段实施与完成条件

### M0 - 远端基线冻结与接口审计

工作：

- 从目标主机重新同步/对比当前 `cage_uav_ws/src`；
- 记录 commit 或生成源码哈希清单；
- 运行现有单测与仿真 acceptance；
- 定位远端 `VLX-Seek`/视觉 v9 的实际输入输出；
- 保存当前规则 demo 的 rosbag、截图和测试结果。

完成条件：

- 明确唯一实施基线；
- 现有失败项有记录；
- 不再把本地旧快照或无关 SCOPE v9 当成当前视觉能力。

### M1 - 不规则网箱与统一 GT

工作：

- 实现共享顶点形变；
- SDF、mesh 和 GT JSON 同源生成；
- world 从 `multi_cage_layout.yaml` 生成；
- 增加 mild/medium/strong preset 与几何校验。

完成条件：

- 相同 seed 完全复现；
- Gazebo、GT、RViz 离线投影对齐；
- 所有 cell 无自交、共边一致；
- 现有相机/任务仿真仍能启动。

### M2 - 消息、GT publisher 与 oracle-noisy 观测

工作：

- 新增 lattice 消息；
- 发布隐藏 GT；
- 从当前 FoV 生成局部 `(i,j)`；
- 实现噪声、漏检、遮挡和边界进入脚本。

完成条件：

- Scout 后端不读取全局真值 ID；
- 观测严格因果；
- 故障脚本和 seed 可复现；
- rosbag 能完整回放。

### M3 - Scout 持久身份

工作：

- Top-K 整数偏移后验；
- identity/visibility 双状态；
- 开放边界 birth；
- 漏检恢复；
- 歧义冻结写入；
- map version 和事件日志。

完成条件：

- 无噪声 100% 正确；
- 1-3 帧漏检不产生 slip；
- confirmed cell 出 FoV 后保留；
- 歧义场景不污染地图。

### M4 - Scout 坐标优化

工作：

- 锚点、名义、相对位移和鲁棒核；
- 输出修正坐标与协方差；
- 地图过期时协方差膨胀；
- 与 GT 自动对比。

完成条件：

- 中等形变下 RMSE 明显优于 nominal；
- 错误关联不会被悄悄写入优化；
- 异常因子可追踪；
- 协方差随证据强弱合理变化。

### M5 - Feeder 身份恢复与任务安全门

工作：

- Mahalanobis GPS 候选；
- 局部邻域 likelihood；
- task transition filter；
- re-entry；
- confirmed/ambiguous/lost/stale map；
- 投料器 gate 和 map snapshot。

完成条件：

- GPS 候选召回达标；
- 三选一相邻格单独通过；
- ambiguous 时绝不投喂；
- 返航重入不会被上一格强行锁死。

### M6 - 接入真实视觉/v9 前端

工作：

- 审计远端模型和数据格式；
- 编写 ROS adapter；
- 同步 image/camera info/pose；
- 输出局部格网和 calibrated confidence；
- 用 oracle 与 vision 在同一 rosbag 对比。

完成条件：

- 视觉输出遵守相同消息合同；
- 后端无需针对 v9 写特殊分支；
- 评测能分清 detector error、association error 和 coordinate error；
- 不使用未来帧。

### M7 - 系统回归、演示和报告

工作：

- 场景矩阵批量运行；
- baseline/ablation；
- RViz/Web 完整显示；
- 生成指标表、关键失败案例和演示视频；
- 回归 circular、rectangular 旧模式和安全链。

完成条件：

- 第 12 节门槛通过；
- 所有结果含配置、seed、map UUID 和代码版本；
- 错误提交与 abstention 分开报告；
- 现有功能无回归。

依赖关系：

```text
M0 -> M1 -> M2 -> M3 -> M4 -> M5 -> M7
                         \-> M6 -----/
```

M6 可以在 M3 接口稳定后并行推进，但不能阻塞 M3-M5 的 oracle 验证。

---

## 16. 推荐运行入口

新 launch 统一提供参数：

```bash
roslaunch cage_simulation persistent_lattice_sim.launch \
  deformation_preset:=medium \
  deformation_seed:=42 \
  observation_source:=oracle_noisy \
  feeder_visual_source:=oracle_noisy \
  start_rviz:=true \
  start_web:=true
```

切换真实视觉时只改：

```bash
observation_source:=vision_v9
feeder_visual_source:=vision_v9
```

批量验收建议：

```bash
rosrun cage_bringup persistent_lattice_acceptance.py \
  --scenario-set standard \
  --seeds 101,102,103,104,105 \
  --output-dir /tmp/cage_lattice_acceptance
```

以上是目标接口，命令在对应阶段实现后才可执行。

---

## 17. 风险与处理

| 风险 | 影响 | 处理 |
|---|---|---|
| 本地快照落后于远端 | 规划文件路径或接口不一致 | M0 先同步和 diff，不直接覆盖 |
| 视频并非用户所指参考 | 形变幅度不准确 | 架构不变，只替换 preset/标定参数 |
| row/col 轴交换或 180 度翻转 | 整体身份错误 | 固定锚点、轴定义，用航向/姿态/历史消歧 |
| 视觉域差 | synthetic 成功、真实失败 | 后端先 oracle 验证，真实视频分层微调 |
| GPS bias 有时间相关性 | 候选门控失真 | 加 bias/random-walk 模型，不只加白噪声 |
| 错误关联污染坐标优化 | 全图坐标漂移 | 身份提交后才加因子，鲁棒核与回滚日志 |
| 多假设数量爆炸 | 实时性下降 | Top-K、合并相同偏移、概率剪枝、超时降级 |
| map 更新移动任务航点 | 飞行安全风险 | 任务版本冻结，超阈值显式 replan |
| ARM/NX mock 分支瞬移 | 误判视觉闭环成功 | mock 仅接口测试；视觉闭环用真实运动或 bag replay |
| 五个网箱复制同一缺陷 | 模型学到场景捷径 | 每个网箱独立 seed，保留同一统计分布 |

---

## 18. 待确认项与默认决策

以下问题不阻塞 M0-M3，先按默认值实施；拿到现场信息后再改配置。

| 待确认 | 默认决策 |
|---|---|
| 用户所说的“不规则网箱视频”是否另有文件 | 当前 demo 只作基线，PDF 第 3 页照片作形变参考 |
| 一个真实 GPS 锚点位于顶点还是 cell 中心 | 固定为 `vertex[0][0]` |
| Scout 与 Feeder 是否两架 UAV | 第一版用同一仿真 UAV 分阶段/回放两角色，topic 保留角色命名空间 |
| 网箱是否随时间形变 | 第一版静态不规则；M7 后再加慢速 drift |
| 是否存在真实物理缺口 | 第一版无，先只注入视觉漏检；有证据后启用 `structural_holes` |
| v9 输出格式与 checkpoint | M0 在目标主机审计，统一适配到 `LocalLatticeObservation` |
| 投喂点取 polygon centroid 还是其他位置 | 第一版取四边形 centroid，可配置 offset |

---

## 19. 最终交付物

完成后应至少具备：

1. 三档可复现的不规则网箱场景和 GT manifest。
2. Scout 持久格网 ROS 节点、地图消息和快照文件。
3. Feeder 相邻候选身份滤波节点与安全拒判。
4. 现有 planner/FSM/投料器/Web/RViz 的兼容接入。
5. oracle-noisy 与 vision-v9 可切换的统一入口。
6. 自动 rosbag、事件日志、批量场景评测与报告。
7. baseline、ablation、失败案例和演示视频。
8. rectangular/circular 旧功能回归结果。

## 20. Definition of Done

只有同时满足以下条件，才算“在仿真里实现了 PDF 的功能”，而不是只画了一个更像真实网箱的模型：

- Gazebo 网箱几何确实不完全规则，且任务 GT 与画面同源；
- Scout 在 cell 进入、离开和漏检时保持永久 `(r,c)`；
- 不确定时保留多假设，不强制写坏地图；
- Scout 修正坐标优于名义规则格网，并输出可信协方差；
- Feeder 能在 GPS 附近相邻候选中恢复 current cell；
- 相邻候选接近时输出 `AMBIGUOUS` 并阻止投喂；
- 返航重入、地图过期和任务中断有明确处理；
- 所有关键指标、seed、版本和失败原因可复现；
- 现有任务模式、圆形任务和安全链没有回归。

---

## 21. 建议立即执行的第一批工作

按以下顺序开始，不等待视觉模型：

1. 在目标主机冻结并同步最新源码，完成 M0。
2. 为 `generate_fish_farm.py` 增加共享顶点形变和 GT JSON。
3. 增加几何校验单测，确认 mild/medium/strong 都无自交。
4. 新建 `persistent_lattice_sim.launch`，不改变现有 circular 默认入口。
5. 定义 lattice ROS 消息与 `/sim_gt` 隔离规则。
6. 完成 `oracle_noisy` 局部观测，先跑漏 1 帧导致 `+-1 slip` 的反例。
7. 实现最小 Scout Top-K 偏移与 confirmed/visibility 状态机。
8. 通过漏检恢复后，再进入坐标优化和 Feeder。

这条路径能最快得到一个可量化、可回放的闭环，同时为远端 v9/视觉代码保留稳定接入点。
