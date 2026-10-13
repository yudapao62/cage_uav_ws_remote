#!/usr/bin/env python3
"""Offline H12 manual takeover timing and no-auto-reentry analysis."""
import argparse
import csv
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

from _analysis_common import ensure_bag, load_config, resolve_result, ros_time, yaml_topic


def last_before(events, stamp):
    values=[item for item in events if item[0] <= stamp]
    return values[-1] if values else None


def first_between(events, start, end, predicate=lambda _m: True):
    return next((item for item in events if start <= item[0] <= end and predicate(item[1])),None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result',help='experiment result directory or raw.bag')
    parser.add_argument('--config')
    parser.add_argument('--rc-mode-channel',type=int,help='one-based RC mode switch channel')
    parser.add_argument('--manual-response-confirmed',action='store_true',help='operator confirmed actual manual aircraft response from video/observation')
    args=parser.parse_args()
    result_dir,bag_path=resolve_result(args.result); ensure_bag(bag_path)
    cfg,_=load_config(args.config); ocfg=cfg['override_analysis']; topics=cfg['topics']
    channel=(args.rc_mode_channel or int(ocfg['rc_mode_channel']))-1
    threshold=float(ocfg['rc_change_threshold_us']); transition_timeout=float(ocfg['mode_transition_timeout_sec']); window=float(ocfg['no_reentry_window_sec'])
    auto=set(str(v) for v in ocfg['auto_modes']); manual=set(str(v) for v in ocfg['manual_modes'])
    rc=[]; states=[]; fsms=[]; velocities=[]; mission_echo=[]
    wanted=[yaml_topic(cfg,k) for k in ('rc_input','mavros_state','fsm_state','local_velocity','mavros_mission_waypoints')]
    with rosbag.Bag(str(bag_path),'r') as bag:
        bag_start=bag.get_start_time(); bag_end=bag.get_end_time()
        for topic,msg,stamp in bag.read_messages(topics=[x for x in wanted if x]):
            ts=ros_time(stamp)
            if topic==topics['rc_input'] and len(msg.channels)>channel:
                rc.append((ts,float(msg.channels[channel]),msg))
            elif topic==topics['mavros_state']: states.append((ts,msg))
            elif topic==topics['fsm_state']: fsms.append((ts,msg))
            elif topic==topics['local_velocity']:
                linear=msg.twist.linear; velocities.append((ts,math.sqrt(linear.x**2+linear.y**2),math.sqrt(linear.x**2+linear.y**2+linear.z**2)))
            elif topic==topics['mavros_mission_waypoints']: mission_echo.append((ts,msg))
    if len(rc)<2: raise SystemExit('Not enough RC samples on {} channel {}'.format(topics['rc_input'],channel+1))
    switches=[]
    for previous,current in zip(rc,rc[1:]):
        if abs(current[1]-previous[1]) >= threshold:
            if not switches or current[0]-switches[-1][0] > 0.25:
                switches.append((current[0],previous[1],current[1]))
    if not switches: raise SystemExit('No RC switch change >= {:.0f} us found on channel {}'.format(threshold,channel+1))
    results=[]
    for index,(t_rc,before_pwm,after_pwm) in enumerate(switches,1):
        state_before=last_before(states,t_rc); mode_before=state_before[1].mode if state_before else ''
        mode_event=first_between(states,t_rc,t_rc+transition_timeout,lambda m: m.mode!=mode_before)
        t_mode=mode_event[0] if mode_event else None; mode_after=mode_event[1].mode if mode_event else ''
        exited_auto=bool(mode_before in auto and mode_after in manual)
        reentry=None
        if t_mode is not None:
            reentry=first_between(states,t_mode+1e-6,t_mode+window,lambda m:m.mode in auto)
        fsm_manual=first_between(fsms,t_rc,t_rc+transition_timeout,lambda m:m.state=='MANUAL_OVERRIDE')
        eval_start=t_mode if t_mode is not None else t_rc; eval_end=eval_start+window
        observed_full=bag_end>=eval_end
        fsm_window=[m for ts,m in fsms if eval_start<=ts<=min(eval_end,bag_end)]
        waypoints=[int(m.current_waypoint) for m in fsm_window]
        stable_progress=(not waypoints) or len(set(waypoints))<=1
        echo_count=sum(1 for ts,_m in mission_echo if eval_start<=ts<=min(eval_end,bag_end))
        speed=[v for ts,v,_v3 in velocities if eval_start<=ts<=min(eval_end,bag_end)]
        max_speed=max(speed) if speed else None
        pass_auto=exited_auto and reentry is None and fsm_manual is not None and stable_progress and observed_full
        status='PASS' if pass_auto and args.manual_response_confirmed else ('FAIL' if not pass_auto else 'INCOMPLETE_NEEDS_MANUAL_RESPONSE_CONFIRMATION')
        results.append({'event':index,'t_rc':t_rc-bag_start,'rc_before_us':before_pwm,'rc_after_us':after_pwm,'mode_before':mode_before,'mode_after':mode_after,'override_latency_sec':None if t_mode is None else t_mode-t_rc,'fsm_manual_latency_sec':None if fsm_manual is None else fsm_manual[0]-t_rc,'auto_reentry':reentry is not None,'auto_reentry_delay_sec':None if reentry is None or t_mode is None else reentry[0]-t_mode,'fsm_waypoint_stable':stable_progress,'window_fully_observed':observed_full,'fc_mission_echo_messages_after':echo_count,'max_horizontal_speed_after_mps':max_speed,'manual_response_confirmed':args.manual_response_confirmed,'result':status})
    fields=list(results[0].keys())
    with (result_dir/'override_metrics.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields); writer.writeheader(); writer.writerows(results)

    fig_dir=result_dir/'figures'; fig_dir.mkdir(parents=True,exist_ok=True)
    t0=bag_start; fig,axes=plt.subplots(5,1,figsize=(12,12),sharex=True)
    axes[0].plot([x[0]-t0 for x in rc],[x[1] for x in rc]); axes[0].set_ylabel('RC ch{} [us]'.format(channel+1)); axes[0].grid(True)
    mode_names=[]
    for _ts,m in states:
        if m.mode not in mode_names: mode_names.append(m.mode)
    mode_map={name:i for i,name in enumerate(mode_names)}
    axes[1].step([x[0]-t0 for x in states],[mode_map[x[1].mode] for x in states],where='post'); axes[1].set_yticks(list(mode_map.values())); axes[1].set_yticklabels(list(mode_map.keys())); axes[1].set_ylabel('FC mode'); axes[1].grid(True)
    fsm_names=[]
    for _ts,m in fsms:
        if m.state not in fsm_names: fsm_names.append(m.state)
    fsm_map={name:i for i,name in enumerate(fsm_names)}
    axes[2].step([x[0]-t0 for x in fsms],[fsm_map[x[1].state] for x in fsms],where='post'); axes[2].set_yticks(list(fsm_map.values())); axes[2].set_yticklabels(list(fsm_map.keys())); axes[2].set_ylabel('FSM'); axes[2].grid(True)
    axes[3].step([x[0]-t0 for x in fsms],[x[1].current_waypoint for x in fsms],where='post',label='FSM current waypoint'); axes[3].scatter([x[0]-t0 for x in mission_echo],[1]*len(mission_echo),marker='x',label='FC mission echo msg'); axes[3].set_ylabel('Mission activity'); axes[3].legend(); axes[3].grid(True)
    if velocities: axes[4].plot([x[0]-t0 for x in velocities],[x[1] for x in velocities],label='Horizontal speed')
    axes[4].set_ylabel('Speed [m/s]'); axes[4].set_xlabel('Time [s]'); axes[4].grid(True)
    for axis in axes:
        for event in results: axis.axvline(event['t_rc'],color='red',alpha=0.35,linestyle='--')
    fig.tight_layout(); fig.savefig(str(fig_dir/'manual_override_timeline.png'),dpi=150); plt.close(fig)

    latencies=np.asarray([r['override_latency_sec'] for r in results if r['override_latency_sec'] is not None],dtype=float)
    stats='N/A'
    if len(latencies): stats='mean={:.4f}, std={:.4f}, min={:.4f}, max={:.4f} s'.format(np.mean(latencies),np.std(latencies,ddof=1) if len(latencies)>1 else 0.0,np.min(latencies),np.max(latencies))
    table='\n'.join('| {event} | {mode_before} → {mode_after} | {override_latency_sec} | {auto_reentry} | {fsm_waypoint_stable} | {result} |'.format(**dict(row,override_latency_sec='N/A' if row['override_latency_sec'] is None else '{:.4f}'.format(row['override_latency_sec']))) for row in results)
    report='''# 人工接管实验报告

## 配置

- Bag: `{bag}`
- RC 模式开关通道：{channel}（一基）
- 开关变化阈值：{threshold:.0f} us
- 模式变化等待：{transition:.1f} s
- 自动重入观察窗口：{window:.1f} s
- 接管延迟统计：{stats}

## 事件

| 事件 | 模式变化 | 接管延迟 [s] | 自动重入 | FSM 序号稳定 | 结果 |
|---:|---|---:|---|---|---|
{table}

## 判定说明

自动判定检查了 RC 开关变化、飞控退出自动模式、FSM 进入 `MANUAL_OVERRIDE`、FSM 当前航点不再推进、以及观察窗口内是否重新进入自动模式。`/mavros/mission/waypoints` 在本架构中是 mission list 回读，不是周期 setpoint；图中的标记只表示该消息出现。

飞机是否真正响应人工摇杆需要结合现场观察、视频、摇杆通道和速度方向人工确认。未传入 `--manual-response-confirmed` 时，即使自动条件满足，结果仍为 `INCOMPLETE_NEEDS_MANUAL_RESPONSE_CONFIRMATION`，不会把“模式已改变”误报为完整 PASS。

## 图表和明细

- [统一时间轴](figures/manual_override_timeline.png)
- 逐事件数据：`override_metrics.csv`
'''.format(bag=bag_path.name,channel=channel+1,threshold=threshold,transition=transition_timeout,window=window,stats=stats,table=table)
    (result_dir/'override_report.md').write_text(report,encoding='utf-8')
    print('Wrote {}'.format(result_dir/'override_report.md')); print('Wrote {}'.format(result_dir/'override_metrics.csv'))


if __name__=='__main__':
    main()
