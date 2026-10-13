# 实机实验前检查清单

## 通用检查

- [ ] 已阅读 `EXPERIMENT_INTERFACE_AUDIT.md`
- [ ] 已运行 `scripts/check_experiment_topics.sh`
- [ ] Ground GPS connected
- [ ] Ground GPS `valid=true`
- [ ] Ground GPS satellites normal
- [ ] MAVROS connected
- [ ] UAV GPS fix valid
- [ ] RC connected
- [ ] RC mode switch detected
- [ ] Manual control verified
- [ ] NX planner running
- [ ] Mission Manager / FSM running
- [ ] Current FC mode known
- [ ] rosbag recording confirmed（目录内出现 `raw.bag.active`）
- [ ] 磁盘剩余空间满足本次时长
- [ ] 系统时钟、ROS 时间正常，没有跳变
- [ ] No critical ROS topic timeout
- [ ] `notes.txt` 已记录场地、天气、固件和操作员

## GPS 对比实验

- [ ] 两套天线同时工作并静止 3–5 min
- [ ] 记录 Ground 相对 UAV 的机械偏移 East/North/Up
- [ ] 已确认 Ground anchor ID（A 或 B）
- [ ] 两套 GPS 同时开始录制
- [ ] 每个位置计划记录至少 10 min
- [ ] 三个地点使用不同 test name

## 航点链路实验

- [ ] 第一阶段已拆桨或保持 disarmed/dry_run
- [ ] 已确认当前采用 MAVLink Mission，不是连续 setpoint
- [ ] `/mission/waypoints` 正常发布
- [ ] `/mavros/mission/waypoints` 可回读
- [ ] `/mavros/mission/reached` 是否可用已记录
- [ ] 飞控日志同步记录
- [ ] WP-01/02/03 目标方位和距离经过人工复核
- [ ] 相对高度和 yaw 单位经过人工复核

## 人工接管实验额外检查

- [ ] Manual takeover tested with propellers OFF
- [ ] 已确认 H12 mode switch 对应 RC 通道编号
- [ ] Auto mode can be exited
- [ ] NX does not automatically re-enter auto mode for 30 s
- [ ] WAIT_ARM 重新请求 AUTO 的已知风险已测试
- [ ] Emergency stop procedure confirmed
- [ ] Flight area clear
- [ ] Level 0 仿真通过
- [ ] Level 1 无桨实机通过
- [ ] 高风险 RC-02/RC-08 尚未越级执行
- [ ] 现场观察员负责确认飞机确实响应人工操纵
