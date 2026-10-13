#!/usr/bin/env python3
"""Continuous fail-closed gate for starting a real vehicle mission."""
import json
import math
import time

import rospy
import rosnode
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import State, RCIn
from cage_msgs.msg import CageGeometry, CageGrid, MissionState
from cage_geo.geodesy import LocalENU


class Precheck:
    def __init__(self):
        self.geometry_mode=str(rospy.get_param('/cage/geometry_mode','rectangular')).strip().lower()
        if self.geometry_mode not in ('rectangular','circular'):
            raise ValueError('invalid /cage/geometry_mode: {}'.format(self.geometry_mode))
        self.fcu = None; self.fcu_at = 0.0
        self.gps = None; self.gps_at = 0.0
        self.rc = None; self.rc_at = 0.0
        self.grid = None; self.home = None; self.geometry = None
        self.feeder_status_at = 0.0
        self.fsm = None; self.beacon = {}; self.beacon_at = 0.0
        self.pub = rospy.Publisher('/cage/realflight/precheck', String, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.on_fcu, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.on_gps, queue_size=1)
        rospy.Subscriber('/mavros/rc/in', RCIn, self.on_rc, queue_size=1)
        rospy.Subscriber('/cage/grid_info', CageGrid, lambda m: setattr(self,'grid',m), queue_size=1)
        rospy.Subscriber('/cage/geometry', CageGeometry, lambda m: setattr(self,'geometry',m), queue_size=1)
        rospy.Subscriber('/cage/home', NavSatFix, lambda m: setattr(self,'home',m), queue_size=1)
        rospy.Subscriber('/mission/state', MissionState, lambda m: setattr(self,'fsm',m), queue_size=1)
        rospy.Subscriber('/cage/beacon/status', String, self.on_beacon, queue_size=1)
        rospy.Subscriber('/feeder/status', Bool, self.on_feeder_status, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.tick)

    def on_fcu(self, msg): self.fcu=msg; self.fcu_at=time.monotonic()
    def on_gps(self, msg): self.gps=msg; self.gps_at=time.monotonic()
    def on_rc(self, msg): self.rc=msg; self.rc_at=time.monotonic()
    def on_beacon(self, msg):
        try: self.beacon=json.loads(msg.data); self.beacon_at=time.monotonic()
        except (TypeError,ValueError): self.beacon={}
    def on_feeder_status(self, _msg): self.feeder_status_at=time.monotonic()

    def tick(self, _event):
        now=time.monotonic()
        gps_ok=(self.gps is not None and now-self.gps_at <= float(rospy.get_param('/safety/gps_timeout',2.0))
            and self.gps.status.status >= NavSatStatus.STATUS_FIX
            and all(math.isfinite(v) for v in (self.gps.latitude,self.gps.longitude,self.gps.altitude))
            and (self.gps.latitude,self.gps.longitude)!=(0.0,0.0))
        fcu_ok=bool(self.fcu and self.fcu.connected and now-self.fcu_at <= float(rospy.get_param('/safety/fcu_timeout',2.0)))
        beacon_ok=(self.beacon.get('geometry_valid') is True and
            now-self.beacon_at <= float(rospy.get_param('/beacon/receive_timeout_sec',2.0)))
        baseline=self.beacon.get('baseline_m')
        grid_alignment=bool(self.grid and baseline is not None and
            abs(self.grid.cols*self.grid.cell_size_x-float(baseline)) <=
            float(rospy.get_param('/cage_grid/max_grid_length_error_m',5.0)))
        center_distance=None
        center_ok=False
        dwell=float(rospy.get_param('/cage/circular/feeding_dwell_sec',5.0))
        if self.geometry_mode=='circular' and self.geometry and gps_ok:
            values=(self.geometry.center_lat,self.geometry.center_lon,self.geometry.center_alt)
            if (self.geometry.geometry_mode=='circular' and all(math.isfinite(v) for v in values) and
                    -90<=values[0]<=90 and -180<=values[1]<=180 and self.geometry.radius_m>0):
                local=LocalENU(self.gps.latitude,self.gps.longitude,self.gps.altitude)
                east,north,_=local.to_enu(*values)
                center_distance=math.hypot(east,north)
                center_ok=center_distance<=float(rospy.get_param('/safety/max_leg_distance',5000.0))
        rc_ok=(self.rc is not None and bool(self.rc.channels) and now-self.rc_at <= 2.0)
        try:
            names=set(rosnode.get_node_names())
            simulation_absent=not any('gazebo' in n or 'feeder_simulator' in n or 'simulation_acceptance' in n for n in names)
        except Exception:
            names=set()
            simulation_absent=False
        checks={'FCU':fcu_ok,'UAV_GPS':gps_ok,'RC_MODE':bool(fcu_ok and self.fcu.mode),
            'SIMULATION_ABSENT':simulation_absent,
            'RC_INPUT':rc_ok if rospy.get_param('/vehicle/require_rc_ready',True) else True,
            'BEACON_A':bool(beacon_ok and self.beacon.get('anchors',{}).get('A',{}).get('valid')),
            'BEACON_B':bool(beacon_ok and self.beacon.get('anchors',{}).get('B',{}).get('valid')),
            'GEOMETRY':beacon_ok,
            'FSM_READY':bool(self.fsm and not self.fsm.manual_override and self.fsm.state in ('IDLE','FINISHED')),
            'AUTO_ARM_FALSE':rospy.get_param('/vehicle/auto_arm',True) is False,
            'FEEDER_REAL':rospy.get_param('/feeding/feeder_mode','')=='real',
            'FEEDER_STATUS':now-self.feeder_status_at <= 2.0,
            'SITE_ALIGNMENT_CONFIRMED':rospy.get_param('/realflight/site_alignment_confirmed',False) is True}
        if self.geometry_mode=='rectangular':
            checks['GRID']=bool(self.grid and self.home and self.grid.rows>0 and self.grid.cols>0 and '/cage_grid_node' in names)
            checks['GRID_ALIGNMENT']=grid_alignment
        else:
            checks['CIRCULAR_TARGET']=bool(center_ok and '/cage_grid_node' in names)
            checks['CIRCULAR_DWELL']=bool(math.isfinite(dwell) and dwell>0)
        ready=all(checks.values())
        rospy.set_param('/realflight/start_ready',ready)
        anchors=self.beacon.get('anchors',{})
        lines=['REALFLIGHT PRECHECK']
        for name,ok in checks.items(): lines.append('{:<26}{}'.format(name,'OK' if ok else 'FAIL'))
        for role in ('A','B'):
            info=anchors.get(role,{})
            lines.append('BEACON {} SAT / AGE         {} / {}'.format(role,info.get('satellites','?'),info.get('age_sec','?')))
        lines.append('BASELINE M                 {}'.format(baseline))
        if self.geometry_mode=='circular': lines.append('CENTER DISTANCE M           {}'.format(center_distance))
        lines.append('RESULT                     {}'.format('READY' if ready else 'FAIL'))
        rospy.loginfo_throttle(5.0,'\n'.join(lines))
        self.pub.publish(String(data=json.dumps({'ready':ready,'checks':checks,'beacon':self.beacon,'geometry_mode':self.geometry_mode,
            'center_distance_m':center_distance,'fcu_mode':self.fcu.mode if self.fcu else '',
            'feeder_mode':rospy.get_param('/feeding/feeder_mode',''),'auto_arm':rospy.get_param('/vehicle/auto_arm',None)},allow_nan=False)))


if __name__=='__main__':
    rospy.init_node('realflight_precheck')
    Precheck()
    rospy.spin()
