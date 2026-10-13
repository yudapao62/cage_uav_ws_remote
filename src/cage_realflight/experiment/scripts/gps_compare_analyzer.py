#!/usr/bin/env python3
"""Offline Ground GPS vs UAV GPS analysis. Reads raw.bag only."""
import argparse
import bisect
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

from cage_geo.geodesy import LocalENU
from _analysis_common import (ensure_bag, finite, load_config, load_metadata,
                              metric, resolve_result, ros_time, write_metrics,
                              yaml_topic)


def extract_samples(bag_path, ground_topic, uav_topic, anchor_id):
    ground, uav = [], []
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, msg, stamp in bag.read_messages(topics=[ground_topic, uav_topic]):
            t = ros_time(stamp)
            if topic == ground_topic:
                if str(msg.anchor_id) != anchor_id or not bool(msg.valid):
                    continue
                if finite(msg.lat, msg.lon, msg.altitude) and (msg.lat != 0.0 or msg.lon != 0.0):
                    ground.append((t, float(msg.lat), float(msg.lon), float(msg.altitude),
                                   int(msg.satellites), int(msg.seq), str(msg.stamp)))
            elif topic == uav_topic:
                status = int(getattr(getattr(msg, 'status', None), 'status', -1))
                if status >= 0 and finite(msg.latitude, msg.longitude, msg.altitude) and (msg.latitude != 0.0 or msg.longitude != 0.0):
                    uav.append((t, float(msg.latitude), float(msg.longitude), float(msg.altitude), status))
    return ground, uav


def to_enu(samples, local):
    return np.asarray([local.to_enu(row[1], row[2], row[3]) for row in samples], dtype=float)


def match_nearest(ground, uav, max_delta):
    ut = [row[0] for row in uav]
    pairs = []
    for gi, grow in enumerate(ground):
        pos = bisect.bisect_left(ut, grow[0])
        candidates = [i for i in (pos - 1, pos) if 0 <= i < len(ut)]
        if not candidates:
            continue
        ui = min(candidates, key=lambda i: abs(ut[i] - grow[0]))
        delta = abs(ut[ui] - grow[0])
        if delta <= max_delta:
            pairs.append((gi, ui, delta))
    return pairs


def axis_metrics(rows, category, data):
    labels = ('east', 'north', 'up')
    for index, label in enumerate(labels):
        values = data[:, index]
        metric(rows, category, 'mean_' + label, np.mean(values), 'm')
        metric(rows, category, 'std_' + label, np.std(values, ddof=1) if len(values) > 1 else 0.0, 'm')
        metric(rows, category, 'peak_to_peak_' + label, np.ptp(values), 'm')
    centered = data[:, :2] - np.mean(data[:, :2], axis=0)
    radius = np.linalg.norm(centered, axis=1)
    metric(rows, category, 'horizontal_deviation_rms', math.sqrt(np.mean(radius ** 2)), 'm')
    metric(rows, category, 'horizontal_deviation_cep50', np.percentile(radius, 50), 'm')
    metric(rows, category, 'horizontal_deviation_cep95', np.percentile(radius, 95), 'm')
    metric(rows, category, 'horizontal_deviation_max', np.max(radius), 'm')


def save_plots(fig_dir, ground, uav, ge, ue, pairs, raw_error, corrected_error):
    fig_dir.mkdir(parents=True, exist_ok=True)
    t0 = min(ground[0][0], uav[0][0])
    gt = np.asarray([r[0] - t0 for r in ground]); ut = np.asarray([r[0] - t0 for r in uav])
    pt = np.asarray([ground[g][0] - t0 for g, _u, _d in pairs])

    plt.figure(figsize=(8, 7)); plt.scatter(ge[:, 0], ge[:, 1], s=10, label='Ground GPS'); plt.scatter(ue[:, 0], ue[:, 1], s=10, label='UAV GPS')
    plt.xlabel('East [m]'); plt.ylabel('North [m]'); plt.axis('equal'); plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(str(fig_dir/'01_enu_scatter.png'), dpi=150); plt.close()

    for number, axis, label in ((2, 0, 'East'), (3, 1, 'North')):
        plt.figure(figsize=(10, 4)); plt.plot(gt, ge[:, axis], label='Ground GPS'); plt.plot(ut, ue[:, axis], label='UAV GPS')
        plt.xlabel('Time [s]'); plt.ylabel('{} [m]'.format(label)); plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(str(fig_dir/'{:02d}_{}_vs_time.png'.format(number,label.lower())), dpi=150); plt.close()

    plt.figure(figsize=(10, 5)); plt.plot(pt, raw_error[:, 0], label='Ground-UAV East'); plt.plot(pt, raw_error[:, 1], label='Ground-UAV North')
    plt.xlabel('Time [s]'); plt.ylabel('Raw error [m]'); plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(str(fig_dir/'04_horizontal_axes_error.png'), dpi=150); plt.close()

    raw_h = np.linalg.norm(raw_error[:, :2], axis=1); corrected_h = np.linalg.norm(corrected_error[:, :2], axis=1)
    plt.figure(figsize=(10, 4)); plt.plot(pt, raw_h, label='Raw'); plt.plot(pt, corrected_h, label='Offset corrected')
    plt.xlabel('Time [s]'); plt.ylabel('Horizontal relative error [m]'); plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(str(fig_dir/'05_horizontal_error.png'), dpi=150); plt.close()

    plt.figure(figsize=(7, 5))
    for values, label in ((raw_h, 'Raw'), (corrected_h, 'Offset corrected')):
        ordered=np.sort(values); cdf=np.arange(1,len(ordered)+1,dtype=float)/len(ordered); plt.plot(ordered,cdf,label=label)
    plt.xlabel('Horizontal relative error [m]'); plt.ylabel('CDF'); plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(str(fig_dir/'06_horizontal_error_cdf.png'), dpi=150); plt.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', help='experiment result directory or raw.bag')
    parser.add_argument('--config')
    parser.add_argument('--ground-anchor-id')
    parser.add_argument('--max-sync-delta', type=float)
    parser.add_argument('--reference', nargs=3, type=float, metavar=('LAT','LON','ALT'))
    args=parser.parse_args()
    result_dir, bag_path=resolve_result(args.result); ensure_bag(bag_path)
    cfg,_=load_config(args.config); meta=load_metadata(result_dir)
    anchor=args.ground_anchor_id or meta.get('ground_anchor_id') or cfg['gps_analysis']['ground_anchor_id']
    max_delta=args.max_sync_delta if args.max_sync_delta is not None else float(cfg['gps_analysis']['max_sync_delta_sec'])
    ground_topic=yaml_topic(cfg,'ground_gps'); uav_topic=yaml_topic(cfg,'uav_gps')
    ground,uav=extract_samples(bag_path,ground_topic,uav_topic,anchor)
    if not ground or not uav:
        raise SystemExit('Need valid samples from both {} anchor={} and {}'.format(ground_topic,anchor,uav_topic))
    reference=tuple(args.reference) if args.reference else tuple(uav[0][1:4])
    local=LocalENU(*reference); ge=to_enu(ground,local); ue=to_enu(uav,local)
    pairs=match_nearest(ground,uav,max_delta)
    if not pairs:
        raise SystemExit('No timestamp pairs within {:.3f} s'.format(max_delta))
    raw=np.asarray([ge[g]-ue[u] for g,u,_d in pairs])
    offsets=np.asarray([float(meta.get('antenna_dx',0.0)),float(meta.get('antenna_dy',0.0)),float(meta.get('antenna_dz',0.0))])
    corrected=raw-offsets
    rows=[]; axis_metrics(rows,'ground_gps',ge); axis_metrics(rows,'uav_gps',ue); axis_metrics(rows,'relative_raw',raw); axis_metrics(rows,'relative_corrected',corrected)
    sync=np.asarray([d for _g,_u,d in pairs]); raw_h=np.linalg.norm(raw[:,:2],axis=1); corrected_h=np.linalg.norm(corrected[:,:2],axis=1)
    metric(rows,'sync','ground_samples',len(ground),'count'); metric(rows,'sync','uav_samples',len(uav),'count'); metric(rows,'sync','matched_samples',len(pairs),'count')
    metric(rows,'sync','unmatched_ground_samples',len(ground)-len(pairs),'count'); metric(rows,'sync','unmatched_uav_samples',len(uav)-len(set(u for _g,u,_d in pairs)),'count')
    metric(rows,'sync','mean_abs_offset',np.mean(sync),'s'); metric(rows,'sync','max_abs_offset',np.max(sync),'s')
    for category,values in (('horizontal_raw',raw_h),('horizontal_corrected',corrected_h)):
        metric(rows,category,'mean',np.mean(values),'m'); metric(rows,category,'rmse',math.sqrt(np.mean(values**2)),'m'); metric(rows,category,'p50',np.percentile(values,50),'m'); metric(rows,category,'p95',np.percentile(values,95),'m'); metric(rows,category,'max',np.max(values),'m')
    write_metrics(result_dir/'gps_metrics.csv',rows); save_plots(result_dir/'figures',ground,uav,ge,ue,pairs,raw,corrected)
    mean_raw=np.mean(raw,axis=0); mean_corrected=np.mean(corrected,axis=0)
    report='''# GPS 对比实验报告

## 数据与时间基准

- Bag: `{bag}`
- Ground: `{ground_topic}`, anchor `{anchor}`，有效样本 {ng}
- UAV: `{uav_topic}`，有效样本 {nu}
- 坐标原点：lat={lat:.9f}, lon={lon:.9f}, alt={alt:.3f}
- 配对：bag receipt time 最近邻，阈值 {sync:.3f} s；匹配 {matched} 组
- 平均/最大同步差：{mean_sync:.4f} / {max_sync:.4f} s

Ground 消息的字符串 `stamp` 只保存为原始字段；为避免混用时间体系，本报告统一使用 rosbag receipt time。

## 稳定性与相对误差

| 指标 | 数值 |
|---|---:|
| Ground East/North 标准差 | {gsde:.3f} / {gsdn:.3f} m |
| UAV East/North 标准差 | {usde:.3f} / {usdn:.3f} m |
| Raw mean East/North/Up bias | {rbe:.3f} / {rbn:.3f} / {rbu:.3f} m |
| Corrected mean East/North/Up bias | {cbe:.3f} / {cbn:.3f} / {cbu:.3f} m |
| Raw horizontal RMSE / P95 / MAX | {rrmse:.3f} / {rp95:.3f} / {rmax:.3f} m |
| Corrected horizontal RMSE / P95 / MAX | {crmse:.3f} / {cp95:.3f} / {cmax:.3f} m |

机械偏移定义为 Ground 相对 UAV 的 East/North/Up，使用值 `{offsets}` m。完整指标见 `gps_metrics.csv`。

## 高度解释限制

`gps_receiver/AnchorFix.altitude` 的大地高/海拔基准没有在消息定义中注明；MAVROS `NavSatFix.altitude` 的实际来源还需结合飞控 GPS 驱动确认。因此 Vertical bias 暂不适合直接作为定位误差结论，水平误差独立有效。

## 图表

1. [ENU 散点](figures/01_enu_scatter.png)
2. [East 随时间](figures/02_east_vs_time.png)
3. [North 随时间](figures/03_north_vs_time.png)
4. [East/North 相对误差](figures/04_horizontal_axes_error.png)
5. [水平相对误差](figures/05_horizontal_error.png)
6. [水平误差 CDF](figures/06_horizontal_error_cdf.png)
'''.format(bag=bag_path.name,ground_topic=ground_topic,anchor=anchor,ng=len(ground),uav_topic=uav_topic,nu=len(uav),lat=reference[0],lon=reference[1],alt=reference[2],sync=max_delta,matched=len(pairs),mean_sync=np.mean(sync),max_sync=np.max(sync),gsde=np.std(ge[:,0],ddof=1) if len(ge)>1 else 0,gsdn=np.std(ge[:,1],ddof=1) if len(ge)>1 else 0,usde=np.std(ue[:,0],ddof=1) if len(ue)>1 else 0,usdn=np.std(ue[:,1],ddof=1) if len(ue)>1 else 0,rbe=mean_raw[0],rbn=mean_raw[1],rbu=mean_raw[2],cbe=mean_corrected[0],cbn=mean_corrected[1],cbu=mean_corrected[2],rrmse=math.sqrt(np.mean(raw_h**2)),rp95=np.percentile(raw_h,95),rmax=np.max(raw_h),crmse=math.sqrt(np.mean(corrected_h**2)),cp95=np.percentile(corrected_h,95),cmax=np.max(corrected_h),offsets=', '.join('{:.3f}'.format(v) for v in offsets))
    (result_dir/'gps_report.md').write_text(report,encoding='utf-8')
    print('Wrote {}'.format(result_dir/'gps_report.md')); print('Wrote {}'.format(result_dir/'gps_metrics.csv'))


if __name__=='__main__':
    main()
