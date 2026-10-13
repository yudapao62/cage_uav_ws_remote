#!/usr/bin/env python3
"""Analyze planner output against the mission echoed back by MAVROS/FC."""
import argparse
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

from cage_geo.geodesy import LocalENU
from _analysis_common import (ensure_bag, finite, load_config, load_metadata,
                              metric, resolve_result, ros_time, write_metrics,
                              yaml_topic)


def latest_before(events, stamp):
    values=[item for item in events if item[0] <= stamp]
    return values[-1] if values else None


def first_after(events, stamp):
    return next((item for item in events if item[0] >= stamp), None)


def enu_xy(local, lat, lon):
    east,north,_up=local.to_enu(float(lat),float(lon),local.home_alt)
    return east,north


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', help='experiment result directory or raw.bag')
    parser.add_argument('--config')
    parser.add_argument('--mission-id', help='select a specific MissionWaypointArray')
    parser.add_argument('--ground-anchor-id')
    args=parser.parse_args()
    result_dir,bag_path=resolve_result(args.result); ensure_bag(bag_path)
    cfg,_=load_config(args.config); meta=load_metadata(result_dir)
    t={key:yaml_topic(cfg,key) for key in cfg['topics']}
    wanted=[v for v in t.values() if v]
    plans=[]; fc_lists=[]; current=[]; fsm=[]; uav=[]; ground=[]; task_req=[]; mission_req=[]; targets=[]; reached=[]
    with rosbag.Bag(str(bag_path),'r') as bag:
        for topic,msg,stamp in bag.read_messages(topics=wanted):
            ts=ros_time(stamp)
            if topic==t['planner_path']: plans.append((ts,msg))
            elif topic==t['mavros_mission_waypoints']: fc_lists.append((ts,msg))
            elif topic==t['current_target']: current.append((ts,msg))
            elif topic==t['fsm_state']: fsm.append((ts,msg))
            elif topic==t['uav_gps'] and finite(msg.latitude,msg.longitude,msg.altitude) and (msg.latitude!=0 or msg.longitude!=0): uav.append((ts,msg))
            elif topic==t['ground_gps'] and bool(msg.valid): ground.append((ts,msg))
            elif topic==t['task_request']: task_req.append((ts,msg))
            elif topic==t['mission_request']: mission_req.append((ts,msg))
            elif topic==t['target_cells']: targets.append((ts,msg))
            elif topic==t['mavros_mission_reached']: reached.append((ts,msg))
    candidates=[x for x in plans if (not args.mission_id or x[1].mission_id==args.mission_id)]
    if not args.mission_id:
        non_preview=[x for x in candidates if x[1].mission_id!='PREVIEW']
        if non_preview: candidates=non_preview
    if not candidates:
        raise SystemExit('No planner message found on {}'.format(t['planner_path']))
    plan_time,plan_msg=candidates[-1]
    uav0=latest_before(uav,plan_time) or first_after(uav,plan_time)
    if uav0 is None:
        raise SystemExit('No UAV GPS sample available for ENU reference')
    local=LocalENU(uav0[1].latitude,uav0[1].longitude,uav0[1].altitude)
    plan_points=np.asarray([(enu_xy(local,w.lat,w.lon)[0],enu_xy(local,w.lat,w.lon)[1],float(w.alt)) for w in plan_msg.waypoints])
    fc_event=first_after(fc_lists,plan_time)
    fc_points=np.empty((0,3)); fc_offset=0; fc_msg=None
    if fc_event:
        fc_time,fc_msg=fc_event
        fc_waypoints=list(fc_msg.waypoints)
        if len(fc_waypoints)==len(plan_msg.waypoints)+1:
            fc_offset=1
        comparable=fc_waypoints[fc_offset:fc_offset+len(plan_msg.waypoints)]
        fc_points=np.asarray([(enu_xy(local,w.x_lat,w.y_long)[0],enu_xy(local,w.x_lat,w.y_long)[1],float(w.z_alt)) for w in comparable])
    else:
        fc_time=None
    anchor_id=args.ground_anchor_id or meta.get('ground_anchor_id') or cfg['gps_analysis']['ground_anchor_id']
    ground_event=latest_before([(ts,msg) for ts,msg in ground if str(msg.anchor_id)==anchor_id],plan_time)
    target_event=latest_before(targets,plan_time)
    target_points=[]
    if target_event:
        target_points=[(enu_xy(local,c.lat,c.lon)[0],enu_xy(local,c.lat,c.lon)[1]) for c in target_event[1].targets]
    selected=[(ts,msg) for ts,msg in current if ts>=plan_time and getattr(msg,'cell_id','')]
    selected_points=[(enu_xy(local,m.lat,m.lon)[0],enu_xy(local,m.lat,m.lon)[1]) for _ts,m in selected if finite(m.lat,m.lon)]
    rows=[]
    metric(rows,'counts','planner_waypoints',len(plan_points),'count'); metric(rows,'counts','fc_echo_waypoints_total',len(fc_msg.waypoints) if fc_msg else 0,'count'); metric(rows,'counts','fc_home_offset',fc_offset,'count'); metric(rows,'counts','current_target_messages',len(selected),'count')
    differences=np.empty((0,3))
    if len(fc_points):
        n=min(len(plan_points),len(fc_points)); differences=fc_points[:n]-plan_points[:n]
        horizontal=np.linalg.norm(differences[:,:2],axis=1); d3=np.linalg.norm(differences,axis=1)
        metric(rows,'consistency','max_horizontal_difference',np.max(horizontal),'m'); metric(rows,'consistency','rmse_horizontal_difference',math.sqrt(np.mean(horizontal**2)),'m'); metric(rows,'consistency','max_3d_numeric_difference',np.max(d3),'m')
    else:
        metric(rows,'consistency','max_horizontal_difference',None,'m')
    prior_task=latest_before(task_req,plan_time); prior_mission=latest_before(mission_req,plan_time); prior_targets=latest_before(targets,plan_time)
    first_selected=first_after(selected,plan_time); first_reached=first_after(reached,plan_time)
    if ground_event: metric(rows,'latency','last_ground_receipt_to_plan',plan_time-ground_event[0],'s')
    if prior_task: metric(rows,'latency','task_request_to_plan',plan_time-prior_task[0],'s')
    if prior_mission: metric(rows,'latency','mission_request_to_plan',plan_time-prior_mission[0],'s')
    if prior_targets: metric(rows,'latency','target_array_to_plan',plan_time-prior_targets[0],'s')
    if fc_time is not None: metric(rows,'latency','plan_to_fc_mission_echo',fc_time-plan_time,'s')
    if first_selected: metric(rows,'latency','plan_to_first_current_cell_report',first_selected[0]-plan_time,'s')
    if first_reached: metric(rows,'latency','plan_to_first_mission_reached',first_reached[0]-plan_time,'s')
    write_metrics(result_dir/'waypoint_metrics.csv',rows)

    fig_dir=result_dir/'figures'; fig_dir.mkdir(parents=True,exist_ok=True)
    plt.figure(figsize=(9,7)); plt.scatter([0],[0],marker='o',s=100,label='UAV initial')
    if ground_event:
        gx,gy=enu_xy(local,ground_event[1].lat,ground_event[1].lon); plt.scatter([gx],[gy],marker='*',s=180,label='Ground GPS {}'.format(anchor_id))
    plt.plot(plan_points[:,0],plan_points[:,1],'-o',label='NX planner')
    if len(fc_points): plt.plot(fc_points[:,0],fc_points[:,1],'--s',fillstyle='none',label='FC mission echo')
    if target_points:
        arr=np.asarray(target_points); plt.scatter(arr[:,0],arr[:,1],marker='x',s=60,label='Target cells')
    if selected_points:
        arr=np.asarray(selected_points); plt.scatter(arr[:,0],arr[:,1],marker='D',s=45,label='Mission current cell reports')
    plt.xlabel('East [m]'); plt.ylabel('North [m]'); plt.axis('equal'); plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(str(fig_dir/'waypoint_chain_enu.png'),dpi=150); plt.close()

    max_diff='N/A' if not len(differences) else '{:.3f} m'.format(np.max(np.linalg.norm(differences[:,:2],axis=1)))
    ground_delay='N/A' if not ground_event else '{:.4f} s (receipt age, not causal latency)'.format(plan_time-ground_event[0])
    fc_delay='N/A' if fc_time is None else '{:.4f} s'.format(fc_time-plan_time)
    report='''# 航点链路实验报告

## 本次数据

- Bag: `{bag}`
- Mission ID: `{mission}`
- Planner 航点数：{plan_count}
- FC 回读航点总数：{fc_count}
- Home 项偏移：{offset}
- Planner → FC 最大水平差：{max_diff}
- 最近 Ground GPS receipt → Planner：{ground_delay}
- Planner → FC mission echo：{fc_delay}

## 链路结论边界

本工程采用 **MAVLink Mission waypoint**，不是 global/local/raw 连续 setpoint。规划结果从 `{planner}` 发布；`MavrosAdapter.upload_mission()` 依次调用 `{clear}`、`{push}`、`{pull}`，并比较回读任务。`{fc_echo}` 是 MAVROS/飞控回读的 mission list，是本报告用于 Level 4 的证据。

rosbag 不能记录 ROS service 的请求体，因此它不能单独证明飞控已经采用并正在导航到某点。飞控实际推进证据还应结合 `{reached}`、FSM 当前序号和飞控日志。Ground GPS 是持续输入，规划由任务请求触发，所以“最近 Ground receipt 到规划”的数值是数据年龄，不应解释为因果处理延迟。

## 坐标与高度

- Planner 和 FC 回读中的经纬度统一转换到以规划前 UAV GPS 为原点的 ENU 后比较。
- `MissionWaypoint.yaw` 由规划器以 degree 生成，并写入 MAVLink waypoint `param4`。
- 上传 frame 为 `FRAME_GLOBAL_REL_ALT`；`z_alt` 是相对高度，不与 `NavSatFix.altitude` 直接相减。
- ENU 只用于离线比较；测试脚本没有重写或发送任何控制目标。

## 图表和明细

- [航点链路 ENU 俯视图](figures/waypoint_chain_enu.png)
- 完整指标：`waypoint_metrics.csv`
'''.format(bag=bag_path.name,mission=plan_msg.mission_id,plan_count=len(plan_points),fc_count=len(fc_msg.waypoints) if fc_msg else 0,offset=fc_offset,max_diff=max_diff,ground_delay=ground_delay,fc_delay=fc_delay,planner=t['planner_path'],clear=cfg['services']['mission_clear'],push=cfg['services']['mission_push'],pull=cfg['services']['mission_pull'],fc_echo=t['mavros_mission_waypoints'],reached=t['mavros_mission_reached'])
    (result_dir/'waypoint_report.md').write_text(report,encoding='utf-8')
    print('Wrote {}'.format(result_dir/'waypoint_report.md')); print('Wrote {}'.format(result_dir/'waypoint_metrics.csv'))


if __name__=='__main__':
    main()
