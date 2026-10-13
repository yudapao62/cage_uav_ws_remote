# 人工接管源码审计

## 1. 自动模式请求位置

工程中允许调用 MAVROS 控制服务的层只有 `cage_mavros_adapter/adapter.py`：

- `arm()` → `/mavros/cmd/arming`
- `set_mode(mode)` → `/mavros/set_mode`
- `start_mission(auto_mode)` → 先 `/mavros/mission/set_current`，再 `set_mode(AUTO)`
- `rtl(mode)` / `request_rtl(mode)` → `set_mode(RTL)`

调用方位于 `cage_mission_fsm/scripts/mission_fsm_node.py`：

- `WAIT_ARM`：若 `auto_arm=true`，每 2 秒尝试预解锁模式和解锁；只要已经 armed 且不在 AUTO，每 2 秒尝试 `start_mission(AUTO)`；
- 进入任务末端：请求 RTL；
- 低电量：请求一次 RTL；
- 超时恢复：按参数重复请求 RTL 并确认 mode。

源码中没有 `OFFBOARD` 或 `GUIDED` 控制链路，默认自动模式为 `AUTO`。实飞配置强制 `auto_arm=false`。

## 2. 已有人工接管保护

`MissionFSM.mode_changed()` 在以下条件同时满足时进入 `MANUAL_OVERRIDE`：

1. `auto_control_started == true`；
2. FSM 位于受控任务或恢复状态；
3. 飞控 mode 进入配置的人工模式集合；
4. 新 mode 与预期自动 mode 不同。

进入后：

- `feeder_enabled=false`；
- `MissionFSMNode.on_state()` 再次发布关闭投料器命令；
- FSM 离开 `WAIT_ARM` 和所有自动推进分支；
- 不会自动恢复任务；
- 只有带 `resume_after_override=true` 的显式新任务请求才允许恢复。

对于已经进入 TAKEOFF/TRANSIT/FEEDING/RTL 等状态的任务，这一设计能够阻止 NX 周期性重新请求 AUTO。

## 3. 发现的风险窗口

### WAIT_ARM 阶段可能重新抢回 AUTO

`mode_changed()` 只有在 `auto_control_started=true` 后才识别人工接管，而该标志在 FSM 确认 armed 且 mode 已经为 AUTO 时才置位。

因此在 `WAIT_ARM` 阶段：

- 飞手已经 armed；
- 飞手选择人工模式；
- `auto_control_started` 仍为 false；
- FSM 不会进入 `MANUAL_OVERRIDE`；
- `tick()` 仍可能每 2 秒调用 `start_mission(AUTO)`。

这会影响 RC-01（Armed/地面）以及 AUTO 切换尚未被 FSM 确认时的接管。第一阶段按要求只记录和报告，不修改核心 FSM。无桨台架测试必须专门覆盖这个窗口。

## 4. 其他限制

- 人工模式识别依赖字符串集合 `STABILIZE/LOITER/POSHOLD/ALT_HOLD/MANUAL`。若 K9/H12 返回其他 mode 名称，FSM 不会识别为接管。
- `/mavros/state` 是飞控确认后的 mode，不等同于 RC 开关原始动作，所以实验必须同时记录 `/mavros/rc/in`。
- 模式改变不能单独证明飞机响应人工摇杆；需要结合 RC 通道、速度、现场观察或视频。
- mission 已上传到飞控后，进入人工模式通常由飞控停止 AUTO 导航；NX 侧不清除任务。重新进入自动模式时飞控如何继续任务必须通过 K9 台架测试确认。

## 5. 实验判据

`manual_override_analyzer.py` 自动检查：

- RC 模式通道变化；
- `/mavros/state` 退出自动 mode；
- FSM 是否进入 `MANUAL_OVERRIDE`；
- FSM current_waypoint 是否停止推进；
- 30 秒观察窗口内是否重新进入自动 mode。

飞机实际响应仍需人工确认。分析命令只有加上 `--manual-response-confirmed` 才可能给出完整 PASS。
