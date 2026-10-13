#!/usr/bin/env python3
"""Read-only three-GPS ENU monitor for RViz, JSON debug and CSV logging."""
import csv
import json
import math
import os
from collections import deque

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from roslib.message import get_message_class
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from cage_geo.geodesy import LocalENU


SOURCE_KEYS = ('gps_a', 'gps_b', 'gps_c')
PAIR_KEYS = (('ab', 'gps_a', 'gps_b'), ('ac', 'gps_a', 'gps_c'), ('bc', 'gps_b', 'gps_c'))


class RunningStats:
    def __init__(self):
        self.count = 0; self.mean = 0.0; self.m2 = 0.0; self.minimum = None; self.maximum = None

    def add(self, value):
        value = float(value); self.count += 1
        delta = value - self.mean; self.mean += delta / self.count; self.m2 += delta * (value - self.mean)
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    @property
    def std(self):
        return math.sqrt(self.m2 / (self.count - 1)) if self.count > 1 else 0.0

    def as_dict(self):
        return {'count': self.count, 'mean': self.mean if self.count else None,
                'std': self.std if self.count else None, 'min': self.minimum, 'max': self.maximum}


def nested_attr(obj, path, default=None):
    try:
        for name in str(path).split('.'):
            obj = getattr(obj, name)
        return obj
    except (AttributeError, TypeError):
        return default


class GPSRelativeVisualizer:
    CSV_FIELDS = ['timestamp','timestamp_a','timestamp_b','timestamp_c',
        'a_lat','a_lon','a_alt','b_lat','b_lon','b_alt','c_lat','c_lon','c_alt',
        'a_e','a_n','a_u','b_e','b_n','b_u','c_e','c_n','c_u',
        'ab_distance_xy','ac_distance_xy','bc_distance_xy',
        'ab_delta_e','ab_delta_n','ab_delta_u','ac_delta_e','ac_delta_n','ac_delta_u',
        'bc_delta_e','bc_delta_n','bc_delta_u','a_status','b_status','c_status']

    def __init__(self):
        self.config = {key: rospy.get_param('~' + key) for key in SOURCE_KEYS}
        self.reference_cfg = rospy.get_param('~reference')
        self.vis = rospy.get_param('~visualization')
        self.trajectory_cfg = rospy.get_param('~trajectory')
        self.require_valid = bool(rospy.get_param('~filter/require_valid', True))
        self.timeout = float(rospy.get_param('~timeout/gps_timeout', 2.0))
        self.sync_warning = float(rospy.get_param('~sync/warning_threshold', 0.2))
        self.ground_truth = rospy.get_param('~ground_truth', {})
        self.frame_id = self.vis.get('frame_id', 'gps_test_enu')
        self.flatten_z = bool(self.vis.get('flatten_z', True))
        self.samples = {key: None for key in SOURCE_KEYS}
        self.trails = {key: deque(maxlen=int(self.trajectory_cfg.get('max_points', 1000))) for key in SOURCE_KEYS}
        self.position_stats = {key: {'east': RunningStats(), 'north': RunningStats()} for key in SOURCE_KEYS}
        self.distance_stats = {pair: RunningStats() for pair, _a, _b in PAIR_KEYS}
        self.pair_signatures = {}; self.csv_signature = None; self.local = None
        self.csv_path = None; self.csv_file = None; self.csv_writer = None; self.csv_rows_since_flush = 0
        self.marker_pub = rospy.Publisher('/gps_test/markers', MarkerArray, queue_size=1)
        self.trail_pub = rospy.Publisher('/gps_test/trails', MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher('/gps_test/relative_status', String, queue_size=1)
        self.subscribers = []
        self._configure_reference()
        self._publish_static_tf()
        for key in SOURCE_KEYS: self._subscribe(key)
        publish_hz = max(0.2, float(self.vis.get('publish_rate_hz', 10.0)))
        console_hz = max(0.1, float(self.vis.get('console_rate_hz', 1.0)))
        rospy.Timer(rospy.Duration(1.0 / publish_hz), self.publish)
        rospy.Timer(rospy.Duration(1.0 / console_hz), self.print_summary)
        rospy.on_shutdown(self.close_csv)

    def _configure_reference(self):
        mode = str(self.reference_cfg.get('mode', 'first_valid')).lower()
        if mode == 'fixed':
            self.local = LocalENU(float(self.reference_cfg['latitude']), float(self.reference_cfg['longitude']), float(self.reference_cfg['altitude']))
            rospy.loginfo('GPS ENU fixed reference: %.9f %.9f %.3f', self.local.home_lat, self.local.home_lon, self.local.home_alt)
        elif mode != 'first_valid':
            raise ValueError('reference.mode must be first_valid or fixed')

    def _publish_static_tf(self):
        transform = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.vis.get('tf_parent_frame', 'world')), child_frame_id=self.frame_id)
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(); self.tf_broadcaster.sendTransform(transform)

    def _subscribe(self, key):
        cfg = self.config[key]; cls = get_message_class(cfg.get('type', ''))
        if cls is None:
            rospy.logerr('%s type is unavailable: %s; source remains INVALID', key, cfg.get('type'))
            return
        self.subscribers.append(rospy.Subscriber(cfg['topic'], cls, self.on_message, callback_args=key, queue_size=20))
        rospy.loginfo('%s (%s) <- %s [%s]', key, cfg.get('name'), cfg['topic'], cfg['type'])

    def on_message(self, msg, key):
        cfg = self.config[key]; selector = cfg.get('selector', {})
        if selector and nested_attr(msg, selector.get('field')) != selector.get('equals'):
            return
        fields = cfg.get('fields', {})
        lat = nested_attr(msg, fields.get('latitude')); lon = nested_attr(msg, fields.get('longitude')); alt = nested_attr(msg, fields.get('altitude'), 0.0)
        if lat is None or lon is None or not all(math.isfinite(float(v)) for v in (lat, lon, alt)) or (float(lat) == 0.0 and float(lon) == 0.0):
            valid = False
        else:
            valid = True
        if fields.get('valid'):
            valid = valid and bool(nested_attr(msg, fields['valid'], False))
        if fields.get('navsat_status'):
            valid = valid and int(nested_attr(msg, fields['navsat_status'], -1)) >= 0
        satellites = nested_attr(msg, fields.get('satellites'), None) if fields.get('satellites') else None
        source_stamp = nested_attr(msg, fields.get('source_stamp'), None) if fields.get('source_stamp') else None
        if hasattr(source_stamp, 'to_sec'): source_stamp = source_stamp.to_sec()
        now = rospy.Time.now()
        sample = {'lat': float(lat) if lat is not None else None, 'lon': float(lon) if lon is not None else None,
                  'alt': float(alt) if alt is not None else None, 'message_valid': bool(valid),
                  'satellites': int(satellites) if satellites is not None else None,
                  'receipt': now, 'source_stamp': str(source_stamp) if source_stamp is not None else ''}
        self.samples[key] = sample
        if self.local is None and str(self.reference_cfg.get('mode', 'first_valid')).lower() == 'first_valid' and key == self.reference_cfg.get('source', 'gps_a') and valid:
            self.local = LocalENU(sample['lat'], sample['lon'], sample['alt'])
            rospy.logwarn('GPS ENU reference locked from %s: %.9f %.9f %.3f', key, sample['lat'], sample['lon'], sample['alt'])
        if self.local is not None and valid:
            position = self.position(key)
            self.trails[key].append(position)
            self.position_stats[key]['east'].add(position[0]); self.position_stats[key]['north'].add(position[1])

    def state(self, key, now=None):
        sample = self.samples[key]
        if sample is None: return 'NO_DATA'
        if self.require_valid and not sample['message_valid']: return 'INVALID'
        now = now or rospy.Time.now()
        if (now - sample['receipt']).to_sec() > self.timeout: return 'STALE'
        return 'VALID'

    def position(self, key):
        sample = self.samples[key]
        altitude = sample['alt'] if bool(self.config[key].get('use_altitude', True)) else self.local.home_alt
        return self.local.to_enu(sample['lat'], sample['lon'], altitude)

    def has_coordinates(self, key):
        sample = self.samples[key]
        return bool(sample and sample['lat'] is not None and sample['lon'] is not None and sample['alt'] is not None and
                    all(math.isfinite(v) for v in (sample['lat'], sample['lon'], sample['alt'])) and
                    (sample['lat'] != 0.0 or sample['lon'] != 0.0))

    def pair_data(self, pair, first, second, positions, states):
        one = self.samples[first]; two = self.samples[second]
        result = {'valid': False, 'sync_delta_sec': None, 'sync_warning': False}
        if one is None or two is None or self.local is None: return result
        delta_t = abs((one['receipt'] - two['receipt']).to_sec()); result['sync_delta_sec'] = delta_t; result['sync_warning'] = delta_t > self.sync_warning
        if states[first] != 'VALID' or states[second] != 'VALID': return result
        a = positions[first]; b = positions[second]; de=b[0]-a[0]; dn=b[1]-a[1]; du=b[2]-a[2]
        xy=math.hypot(de,dn); d3=math.sqrt(de*de+dn*dn+du*du)
        result.update({'valid': True, 'delta_e': de, 'delta_n': dn, 'delta_u': du, 'distance_xy': xy, 'distance_3d': d3})
        gt = self.ground_truth.get(pair, {})
        if bool(gt.get('enable', False)):
            result['ground_truth_distance'] = float(gt.get('distance', 0.0)); result['ground_truth_error'] = xy-result['ground_truth_distance']
        signature=(one['receipt'].to_nsec(),two['receipt'].to_nsec())
        if not result['sync_warning'] and self.pair_signatures.get(pair) != signature:
            self.pair_signatures[pair]=signature; self.distance_stats[pair].add(xy)
        result['stats'] = self.distance_stats[pair].as_dict()
        return result

    def marker(self, marker_id, namespace, marker_type, position=(0,0,0), scale=(1,1,1), color=(1,1,1,1), text=''):
        marker=Marker(header=Header(stamp=rospy.Time.now(),frame_id=self.frame_id),ns=namespace,id=marker_id,type=marker_type,action=Marker.ADD)
        marker.pose.position=Point(*position); marker.pose.orientation.w=1.0
        marker.scale.x,marker.scale.y,marker.scale.z=scale; marker.color.r,marker.color.g,marker.color.b,marker.color.a=color; marker.text=text
        return marker

    def display_position(self, position):
        return (position[0],position[1],0.0 if self.flatten_z else position[2])

    def publish(self, _event):
        self.refresh_csv_path()
        now=rospy.Time.now(); states={key:self.state(key,now) for key in SOURCE_KEYS}
        positions={key:self.position(key) for key in SOURCE_KEYS if self.local is not None and self.has_coordinates(key)}
        pairs={name:self.pair_data(name,a,b,positions,states) for name,a,b in PAIR_KEYS}
        self.publish_markers(states,positions,pairs); self.publish_trails(); status=self.status_dict(now,states,positions,pairs)
        self.status_pub.publish(String(data=json.dumps(status,allow_nan=False,sort_keys=True))); self.write_csv(now,states,positions,pairs)

    def publish_markers(self, states, positions, pairs):
        markers=[]; marker_id=0
        markers.append(Marker(header=Header(stamp=rospy.Time.now(),frame_id=self.frame_id),action=Marker.DELETEALL)); marker_id+=1
        grey=(0.45,0.45,0.45,0.65)
        for key in SOURCE_KEYS:
            cfg=self.config[key]; sample=self.samples[key]
            if key not in positions: continue
            pos=self.display_position(positions[key]); color=tuple(cfg.get('color',[1,1,1,1])) if states[key]=='VALID' else grey
            markers.append(self.marker(marker_id,'gps_points',Marker.SPHERE,pos,(self.vis.get('point_scale',0.8),)*3,color)); marker_id+=1
            p=positions[key]; stat=self.position_stats[key]; sat='N/A' if sample['satellites'] is None else str(sample['satellites'])
            text='{}\n{}\nE {:+.3f}  N {:+.3f}  U {:+.3f} m'.format(cfg.get('name',key),states[key],p[0],p[1],p[2])
            if self.vis.get('show_satellites',True): text+='\nSat '+sat
            text+='\nMean E/N {:+.3f}/{:+.3f}\nStd E/N {:.3f}/{:.3f}'.format(stat['east'].mean,stat['north'].mean,stat['east'].std,stat['north'].std)
            markers.append(self.marker(marker_id,'gps_labels',Marker.TEXT_VIEW_FACING,(pos[0],pos[1],pos[2]+1.0),(0,0,self.vis.get('text_height',0.6)),color,text)); marker_id+=1
        for pair,a,b in PAIR_KEYS:
            if a not in positions or b not in positions: continue
            pa=self.display_position(positions[a]); pb=self.display_position(positions[b]); data=pairs[pair]
            if data['valid']:
                line=self.marker(marker_id,'distance_lines',Marker.LINE_STRIP,scale=(self.vis.get('line_width',0.08),0,0),color=(1.0,0.85,0.15,0.9)); marker_id+=1
                line.points=[Point(*pa),Point(*pb)]; markers.append(line)
                text='{}-{}  XY {:.3f} m'.format(pair[0].upper(),pair[1].upper(),data['distance_xy'])
                if self.vis.get('show_relative_position',True): text+='\nΔE {:+.3f}  ΔN {:+.3f}'.format(data['delta_e'],data['delta_n'])
                if self.vis.get('show_altitude_difference',True): text+='  ΔH {:+.3f}'.format(data['delta_u'])
                stats=data['stats']; text+='\nMean {:.3f}  Std {:.3f}'.format(stats['mean'],stats['std'])
                if data.get('sync_warning'): text+='\nSYNC WARNING {:.0f} ms'.format(1000*data['sync_delta_sec'])
                if 'ground_truth_distance' in data: text+='\nGT {:.3f}  Error {:+.3f}'.format(data['ground_truth_distance'],data['ground_truth_error'])
                color=(1.0,0.35,0.15,1.0) if data.get('sync_warning') else (1.0,0.95,0.55,1.0)
            else:
                text='{}-{}  N/A\n{} / {}'.format(pair[0].upper(),pair[1].upper(),states[a],states[b]); color=grey
            midpoint=tuple((pa[i]+pb[i])/2.0 for i in range(3)); midpoint=(midpoint[0],midpoint[1],midpoint[2]+0.5)
            markers.append(self.marker(marker_id,'distance_text',Marker.TEXT_VIEW_FACING,midpoint,(0,0,self.vis.get('text_height',0.6)),color,text)); marker_id+=1
        if pairs['ac'].get('valid'):
            pa=self.display_position(positions['gps_a']); pc=self.display_position(positions['gps_c'])
            arrow=self.marker(marker_id,'error_vectors',Marker.ARROW,scale=(0.12,0.24,0.34),color=(0.95,0.20,0.85,0.95)); marker_id+=1; arrow.points=[Point(*pa),Point(*pc)]; markers.append(arrow)
        axis_length=float(self.vis.get('axis_length',3.0))
        for end,color,label in (((axis_length,0,0),(1,0,0,1),'E'),((0,axis_length,0),(0,1,0,1),'N'),((0,0,axis_length),(0.1,0.5,1,1),'U')):
            arrow=self.marker(marker_id,'enu_axes',Marker.ARROW,scale=(0.08,0.16,0.22),color=color); marker_id+=1; arrow.points=[Point(0,0,0),Point(*end)]; markers.append(arrow)
            markers.append(self.marker(marker_id,'enu_axes',Marker.TEXT_VIEW_FACING,end,(0,0,0.5),color,label)); marker_id+=1
        panel='GPS RELATIVE POSITION TEST\nX=East  Y=North  Z=Up'
        for key in SOURCE_KEYS:
            sample=self.samples[key]; sat='N/A' if not sample or sample['satellites'] is None else sample['satellites']; panel+='\n{}: {}  Sat {}'.format(key[-1].upper(),states[key],sat)
        for pair,_a,_b in PAIR_KEYS:
            data=pairs[pair]; panel+='\n{}: {}'.format(pair.upper(),'{:.3f} m'.format(data['distance_xy']) if data.get('valid') else 'N/A')
        markers.append(self.marker(marker_id,'status_panel',Marker.TEXT_VIEW_FACING,tuple(self.vis.get('status_position',[-4,-4,5])),(0,0,self.vis.get('text_height',0.6)),(1,1,1,1),panel))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def publish_trails(self):
        markers=[Marker(header=Header(stamp=rospy.Time.now(),frame_id=self.frame_id),action=Marker.DELETEALL)]
        if not self.trajectory_cfg.get('enable',True): self.trail_pub.publish(MarkerArray(markers=markers)); return
        for index,key in enumerate(SOURCE_KEYS):
            marker=self.marker(index+1,'gps_trails',Marker.POINTS,scale=(self.trajectory_cfg.get('point_scale',0.10),self.trajectory_cfg.get('point_scale',0.10),0),color=tuple(self.config[key].get('color',[1,1,1,1])))
            marker.points=[Point(*self.display_position(p)) for p in self.trails[key]]; markers.append(marker)
        self.trail_pub.publish(MarkerArray(markers=markers))

    def status_dict(self, now, states, positions, pairs):
        result={'header':{'receipt_time':now.to_sec(),'frame_id':self.frame_id},'reference':None,'sources':{},'pairs':pairs}
        if self.local: result['reference']={'latitude':self.local.home_lat,'longitude':self.local.home_lon,'altitude':self.local.home_alt}
        for key in SOURCE_KEYS:
            sample=self.samples[key]; item={'name':self.config[key].get('name',key),'status':states[key]}
            if sample:
                item.update({'latitude':sample['lat'],'longitude':sample['lon'],'altitude':sample['alt'],'satellites':sample['satellites'],'receipt_time':sample['receipt'].to_sec(),'source_stamp':sample['source_stamp'],'age_sec':(now-sample['receipt']).to_sec()})
            if key in positions:
                item['enu']={'east':positions[key][0],'north':positions[key][1],'up':positions[key][2]}; item['stats']={name:stat.as_dict() for name,stat in self.position_stats[key].items()}
            result['sources'][key]=item
        return result

    def refresh_csv_path(self):
        requested=str(rospy.get_param('~csv/path','') or '')
        if requested==self.csv_path: return
        self.close_csv(); self.csv_path=requested
        if not requested: return
        directory=os.path.dirname(os.path.abspath(requested)); os.makedirs(directory,exist_ok=True)
        is_new=not os.path.exists(requested) or os.path.getsize(requested)==0
        self.csv_file=open(requested,'a',newline=''); self.csv_writer=csv.DictWriter(self.csv_file,fieldnames=self.CSV_FIELDS)
        if is_new: self.csv_writer.writeheader(); self.csv_file.flush()
        rospy.loginfo('GPS relative CSV logging: %s',requested)

    def close_csv(self):
        if self.csv_file: self.csv_file.flush(); self.csv_file.close()
        self.csv_file=None; self.csv_writer=None; self.csv_rows_since_flush=0

    def write_csv(self, now, states, positions, pairs):
        if not self.csv_writer or self.local is None or any(k not in positions for k in SOURCE_KEYS): return
        signature=tuple(self.samples[k]['receipt'].to_nsec() for k in SOURCE_KEYS)
        if signature==self.csv_signature: return
        self.csv_signature=signature; row={'timestamp':now.to_sec()}
        for short,key in zip(('a','b','c'),SOURCE_KEYS):
            sample=self.samples[key]; pos=positions[key]
            row.update({'timestamp_'+short:sample['receipt'].to_sec(),short+'_lat':sample['lat'],short+'_lon':sample['lon'],short+'_alt':sample['alt'],short+'_e':pos[0],short+'_n':pos[1],short+'_u':pos[2],short+'_status':states[key]})
        for pair,_a,_b in PAIR_KEYS:
            data=pairs[pair]
            row[pair+'_distance_xy']=data.get('distance_xy',''); row[pair+'_delta_e']=data.get('delta_e',''); row[pair+'_delta_n']=data.get('delta_n',''); row[pair+'_delta_u']=data.get('delta_u','')
        self.csv_writer.writerow(row); self.csv_rows_since_flush+=1
        if self.csv_rows_since_flush>=int(rospy.get_param('~csv/flush_every_rows',10)):
            self.csv_file.flush(); self.csv_rows_since_flush=0

    def print_summary(self, _event):
        now=rospy.Time.now(); states={key:self.state(key,now) for key in SOURCE_KEYS}
        if self.local is None:
            rospy.loginfo('GPS Relative Monitor: waiting for ENU reference from %s',self.reference_cfg.get('source','gps_a')); return
        positions={key:self.position(key) for key in SOURCE_KEYS if self.has_coordinates(key)}
        pairs={name:self.pair_data(name,a,b,positions,states) for name,a,b in PAIR_KEYS}
        lines=['GPS Relative Position Monitor | reference {} {:.9f},{:.9f}'.format(self.reference_cfg.get('source','fixed'),self.local.home_lat,self.local.home_lon)]
        for key in SOURCE_KEYS:
            if key in positions:
                p=positions[key]; lines.append('{} {} E={:+.3f} N={:+.3f} U={:+.3f} Sat={}'.format(self.config[key].get('name'),states[key],p[0],p[1],p[2],self.samples[key]['satellites'] if self.samples[key]['satellites'] is not None else 'N/A'))
            else: lines.append('{} {}'.format(self.config[key].get('name'),states[key]))
        for pair,_a,_b in PAIR_KEYS:
            data=pairs[pair]; lines.append('{} {}'.format(pair.upper(),'XY={:.3f}m dE={:+.3f} dN={:+.3f} dt={:.0f}ms{}'.format(data['distance_xy'],data['delta_e'],data['delta_n'],1000*data['sync_delta_sec'],' SYNC_WARNING' if data['sync_warning'] else '') if data.get('valid') else 'N/A'))
        rospy.loginfo('\n'.join(lines))


if __name__ == '__main__':
    rospy.init_node('gps_relative_visualizer')
    GPSRelativeVisualizer()
    rospy.spin()
