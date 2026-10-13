#!/usr/bin/env python3
import math
import time
import uuid
import rospy
from std_msgs.msg import Empty, Header, String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import State
from cage_msgs.msg import (CageGeometry, CageGrid, CellTarget, CellTargetArray,
                           FeedingTaskRequest, FeedingTaskStatus, MissionRequest,
                           MissionState, PersistentLattice,
                           ScoutMapActivationStatus)
from cage_msgs.srv import ManageScoutMapLease, ManageScoutMapLeaseRequest
from cage_grid.model import CageGridModel, normalize_geometry_mode
from cage_geo.geodesy import LocalENU
from cage_feeding_task.persistent_map_adapter import PersistentMapTargetModel
from cage_feeding_task.target_generator import generate_targets


class FeedingTaskManager:
    def __init__(self):
        p=lambda n,d: rospy.get_param('/feeding/'+n,d)
        self.geometry_mode=normalize_geometry_mode(rospy.get_param('/cage/geometry_mode','rectangular'))
        self.use_persistent_map=bool(rospy.get_param('/persistent_lattice/use_for_tasks',False))
        self.active_status_timeout=float(rospy.get_param(
            '/scout_map_registry/active_status_timeout_sec',3.0))
        self.lease_ttl=float(rospy.get_param('/scout_map_registry/lease_ttl_sec',5.0))
        self.beacon_mode=rospy.get_param('/cage_grid/geometry_source','fixed')=='beacon'
        self.model=None if (self.beacon_mode or self.geometry_mode=='circular') else CageGridModel(int(p('active_cage_id',0)), int(rospy.get_param('/cage_grid/rows',6)), int(rospy.get_param('/cage_grid/cols',10)),
            rospy.get_param('/cage_grid/origin_x',70.0), rospy.get_param('/cage_grid/origin_y',-6.0), rospy.get_param('/cage_grid/origin_z',0.0),
            rospy.get_param('/cage_grid/cell_size_x',12.0), rospy.get_param('/cage_grid/cell_size_y',12.0),
            math.radians(rospy.get_param('/cage_grid/yaw_deg',0.0)),
            (rospy.get_param('/cage_grid/reference_lat',-35.363261), rospy.get_param('/cage_grid/reference_lon',149.165230), rospy.get_param('/cage_grid/reference_alt',584.0)))
        self.max_repeat=int(p('max_repeat_count',20)); self.max_targets=int(p('max_targets_per_mission',200)); self.interval=float(p('repeat_interval_sec',3.0))
        self.task=None; self.targets=[]; self.current_repeat=0; self.status_state='TASK_IDLE'; self.mission_state=''; self.fcu=State(); self.gps=None; self.repeat_due=0.0; self.last_mission=''; self.batch_seq=0; self.current_cell_id=''
        self.grid_info=None; self.home=None; self.beacon_status={}; self.geometry=None
        self.latest_persistent_model=None
        self.active_map_status=None; self.last_active_status_monotonic=0.0
        self.task_map_model=None; self.pending_target_model=None
        self.map_lease_id=''; self.last_lease_renew=0.0
        self.status_pub=rospy.Publisher('/feeding/task_status',FeedingTaskStatus,queue_size=1,latch=True)
        self.target_pub=rospy.Publisher('/feeding/target_cells',CellTargetArray,queue_size=1,latch=True)
        self.current_pub=rospy.Publisher('/feeding/current_cell',CellTarget,queue_size=1,latch=True)
        self.mission_pub=rospy.Publisher('/mission/request',MissionRequest,queue_size=1)
        self.mission_cancel_pub=rospy.Publisher('/mission/cancel',String,queue_size=1)
        rospy.Subscriber('/feeding/task_request',FeedingTaskRequest,self.on_request,queue_size=1)
        rospy.Subscriber('/feeding/task_preview',FeedingTaskRequest,self.on_preview,queue_size=1)
        rospy.Subscriber('/feeding/task_cancel',Empty,self.on_cancel,queue_size=1)
        rospy.Subscriber('/mission/state',MissionState,self.on_mission,queue_size=1)
        rospy.Subscriber('/mavros/state',State,lambda m:setattr(self,'fcu',m),queue_size=1)
        rospy.Subscriber('/mavros/global_position/global',NavSatFix,lambda m:setattr(self,'gps',m),queue_size=1)
        rospy.Subscriber('/cage/geometry',CageGeometry,self.on_geometry,queue_size=1)
        if self.use_persistent_map:
            rospy.Subscriber('/scout/lattice_map',PersistentLattice,self.on_persistent_map,queue_size=1)
            rospy.Subscriber('/scout/active_map_status',ScoutMapActivationStatus,
                             self.on_active_map_status,queue_size=1)
            self.manage_map_lease=rospy.ServiceProxy(
                '/scout/manage_map_lease',ManageScoutMapLease)
        if self.beacon_mode:
            rospy.Subscriber('/cage/grid_info',CageGrid,self.on_grid,queue_size=1)
            rospy.Subscriber('/cage/home',NavSatFix,self.on_home,queue_size=1)
            rospy.Subscriber('/cage/beacon/status',String,self.on_beacon_status,queue_size=1)
        rospy.Timer(rospy.Duration(0.2),self.tick); self.publish_status('Waiting for task')

    def on_geometry(self,msg):
        if not msg.geometry_mode or msg.geometry_mode == self.geometry_mode:
            self.geometry=msg

    def on_persistent_map(self,msg):
        try:
            self.latest_persistent_model=PersistentMapTargetModel.from_message(msg)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0,'Ignoring unusable Scout map: %s',exc)

    def on_active_map_status(self,msg):
        self.active_map_status=msg
        self.last_active_status_monotonic=time.monotonic()

    @staticmethod
    def _model_pin(model):
        return (model.map_uuid,model.map_version,model.map_sha256,
                model.manifest_sha256)

    def _active_map_reason(self,target_model):
        status=self.active_map_status
        if status is None: return 'NO_ACTIVE_MAP_STATUS'
        if time.monotonic()-self.last_active_status_monotonic > self.active_status_timeout:
            return 'REGISTRY_HEARTBEAT_STALE'
        if status.status != ScoutMapActivationStatus.STATUS_ACTIVE:
            return status.reason_code or 'NO_ACTIVE_MAP'
        if not status.frozen or not status.compatible: return 'ACTIVE_MAP_NOT_COMPATIBLE'
        status_pin=(status.map_uuid,int(status.map_version),status.map_sha256,
                    status.manifest_sha256)
        if status_pin != self._model_pin(target_model): return 'ACTIVE_MAP_STATUS_MISMATCH'
        return ''

    def _acquire_lease(self,req,model):
        try:
            rospy.wait_for_service('/scout/manage_map_lease',timeout=2.0)
            response=self.manage_map_lease(
                operation=ManageScoutMapLeaseRequest.OP_ACQUIRE,
                feeding_task_id=req.task_id,lease_id='',
                map_uuid=model.map_uuid,map_version=model.map_version,
                map_sha256=model.map_sha256,
                manifest_sha256=model.manifest_sha256,
                requested_ttl_sec=self.lease_ttl)
        except (rospy.ROSException,rospy.ServiceException) as exc:
            return 'MAP_LEASE_SERVICE_FAILED:{}'.format(exc)
        if not response.success: return response.reason_code or response.message
        self.map_lease_id=response.lease_id; self.last_lease_renew=time.monotonic()
        return ''

    def _release_lease(self):
        if not self.map_lease_id or self.task is None: self.map_lease_id=''; return
        try:
            self.manage_map_lease(
                operation=ManageScoutMapLeaseRequest.OP_RELEASE,
                feeding_task_id=self.task.task_id,lease_id=self.map_lease_id,
                map_uuid='',map_version=0,map_sha256='',manifest_sha256='',
                requested_ttl_sec=self.lease_ttl)
        except rospy.ServiceException as exc:
            rospy.logwarn('Map lease release failed: %s',exc)
        self.map_lease_id=''

    def _renew_lease(self):
        if not self.map_lease_id or self.task is None: return True
        try:
            response=self.manage_map_lease(
                operation=ManageScoutMapLeaseRequest.OP_RENEW,
                feeding_task_id=self.task.task_id,lease_id=self.map_lease_id,
                map_uuid='',map_version=0,map_sha256='',manifest_sha256='',
                requested_ttl_sec=self.lease_ttl)
        except rospy.ServiceException:
            return False
        if response.success:
            self.last_lease_renew=time.monotonic(); return True
        return False

    def on_grid(self,msg):
        self.grid_info=msg; self.update_model()

    def on_home(self,msg):
        self.home=msg; self.update_model()

    def on_beacon_status(self,msg):
        try: self.beacon_status=__import__('json').loads(msg.data)
        except (ValueError,TypeError): self.beacon_status={}

    def update_model(self):
        if self.geometry_mode == 'circular' or not self.beacon_mode or self.grid_info is None or self.home is None: return
        if self.status_state in ('TASK_READY','TASK_EXECUTING','TASK_REPEAT_WAIT','TASK_MANUAL_OVERRIDE'): return
        g=self.grid_info
        self.model=CageGridModel(g.cage_id,g.rows,g.cols,g.origin_x,g.origin_y,g.origin_z,
            g.cell_size_x,g.cell_size_y,g.yaw,(self.home.latitude,self.home.longitude,self.home.altitude))

    def valid_ready(self, req):
        if self.beacon_mode and not self.beacon_status.get('geometry_valid',False): return 'BEACON_NOT_VALID'
        if self.beacon_mode and rospy.Time.now().to_sec()-float(self.beacon_status.get('published_at',0)) > float(rospy.get_param('/beacon/receive_timeout_sec',2.0)): return 'BEACON_STATUS_STALE'
        if req.repeat_count < 1 or req.repeat_count > self.max_repeat: return 'INVALID_REPEAT_COUNT'
        if not self.fcu.connected: return 'FCU_NOT_CONNECTED'
        if self.gps is None or self.gps.status.status < NavSatStatus.STATUS_FIX: return 'GPS_NOT_VALID'
        if self.mission_state in ('TAKEOFF','TRANSIT','SCOUTING','FINALIZING_MAP',
                                  'FEEDING','FEEDING_DWELL','EXIT','RTL','LAND','UPLOAD_MISSION','WAIT_ARM',
                                  'TIMEOUT_ABORT','RECOVERY_RTL','RECOVERY_RTL_VERIFY','RECOVERY_RETURNING',
                                  'RECOVERY_LANDING','RECOVERY_LANDED'):
            return 'MISSION_BUSY'
        if self.geometry_mode == 'circular':
            if self.geometry is None: return 'CIRCULAR_GEOMETRY_NOT_READY'
            if req.cage_id != self.geometry.cage_id: return 'ONLY_ACTIVE_CAGE_SUPPORTED'
            values=(self.geometry.center_lat,self.geometry.center_lon,self.geometry.center_alt,self.geometry.diameter_m)
            if (not all(math.isfinite(v) for v in values) or not (-90 <= values[0] <= 90) or
                    not (-180 <= values[1] <= 180) or self.geometry.diameter_m <= 0):
                return 'CIRCULAR_CENTER_INVALID'
            dwell=float(rospy.get_param('/cage/circular/feeding_dwell_sec',5.0))
            if not math.isfinite(dwell) or dwell <= 0: return 'CIRCULAR_DWELL_INVALID'
            local=LocalENU(self.gps.latitude,self.gps.longitude,self.gps.altitude)
            east,north,_=local.to_enu(self.geometry.center_lat,self.geometry.center_lon,self.geometry.center_alt)
            distance=math.hypot(east,north)
            if distance > float(rospy.get_param('/safety/max_leg_distance',5000.0)):
                return 'CIRCULAR_TARGET_TOO_FAR'
            rospy.loginfo('[CIRCULAR] center=(%.8f, %.8f), target distance=%.2f m',
                          self.geometry.center_lat,self.geometry.center_lon,distance)
            return []
        target_model=self.latest_persistent_model if self.use_persistent_map else self.model
        if target_model is None: return 'SCOUT_MAP_NOT_READY' if self.use_persistent_map else 'GRID_NOT_READY'
        if self.use_persistent_map:
            active_reason=self._active_map_reason(target_model)
            if active_reason: return active_reason
        if req.cage_id != target_model.cage_id: return 'ONLY_ACTIVE_CAGE_SUPPORTED'
        try:
            cells=generate_targets(target_model,req,(target_model.local.to_enu(self.gps.latitude,self.gps.longitude,self.gps.altitude)[:2]))
        except (ValueError,TypeError) as exc: return str(exc)
        if len(cells)>self.max_targets: return 'TOO_MANY_TARGETS'
        self.pending_target_model=target_model
        return cells

    def on_preview(self, req):
        result=self.valid_ready(req)
        if isinstance(result,str): self.status_state='TASK_ERROR'; self.publish_status(result); return
        self.task=req; self.targets=result; self.task_map_model=self.pending_target_model; self.current_repeat=0; self.status_state='TASK_PREVIEW'; self.publish_targets(); self.publish_status('Preview only; no mission or arming requested')
        if self.geometry_mode == 'circular':
            self.mission_pub.publish(MissionRequest(header=Header(stamp=rospy.Time.now()),mission_id='PREVIEW',
                target_cage_id=self.active_cage_id(),start=False,resume_after_override=False,rtl_after_mission=True,
                feed_relative_alt=req.feed_relative_alt,transit_relative_alt=req.transit_relative_alt,
                mission_type=MissionRequest.MISSION_FEEDER,auto_land=True))

    def on_request(self, req):
        if self.beacon_mode and (rospy.get_param('/feeding/feeder_mode','dry_run') != 'real' or rospy.get_param('/vehicle/auto_arm',True)):
            self.status_state='TASK_ERROR'; self.publish_status('REAL_FEEDER_REQUIRED_AND_AUTO_ARM_MUST_BE_FALSE'); return
        if self.beacon_mode and not rospy.get_param('/realflight/start_ready',False):
            self.status_state='TASK_ERROR'; self.publish_status('REALFLIGHT_PRECHECK_FAILED'); return
        result=self.valid_ready(req)
        if isinstance(result,str): self.status_state='TASK_ERROR'; self.publish_status(result); return
        if self.use_persistent_map:
            lease_error=self._acquire_lease(req,self.pending_target_model)
            if lease_error:
                self.status_state='TASK_ERROR'; self.publish_status(lease_error); return
        self.task=req; self.targets=result; self.task_map_model=self.pending_target_model; self.current_repeat=1; self.status_state='TASK_READY'; self.start_round()

    def start_round(self):
        if not self.task: return
        self.status_state='TASK_EXECUTING'; self.last_mission='feed-{}-r{}-{}'.format(self.task.task_id or uuid.uuid4().hex[:8],self.current_repeat,int(time.time()))
        self.publish_targets()
        rospy.sleep(rospy.Duration(0.3))
        self.publish_status('Starting repeat {}/{}'.format(self.current_repeat,self.task.repeat_count))
        model=self.task_map_model
        self.mission_pub.publish(MissionRequest(header=Header(stamp=rospy.Time.now()),mission_id=self.last_mission,target_cage_id=self.active_cage_id(),start=True,resume_after_override=False,rtl_after_mission=self.current_repeat>=self.task.repeat_count,feed_relative_alt=self.task.feed_relative_alt,transit_relative_alt=self.task.transit_relative_alt,mission_type=MissionRequest.MISSION_FEEDER,auto_land=True,
            task_map_uuid=model.map_uuid if model else '',task_map_version=model.map_version if model else 0,
            task_map_sha256=model.map_sha256 if model else '',task_manifest_sha256=model.manifest_sha256 if model else ''))

    def on_cancel(self,_):
        if self.task:
            self._release_lease()
            self.status_state='TASK_CANCELLED'; self.publish_status('Cancelled; feeder is disabled by mission FSM/manual hold')
            self.task=None; self.targets=[]; self.task_map_model=None
            self.update_model()

    def on_mission(self,msg):
        self.mission_state=msg.state
        self.current_cell_id=msg.current_cell_id
        if msg.state in ('TIMEOUT_ABORT','RECOVERY_RTL','RECOVERY_RTL_VERIFY','RECOVERY_RETURNING',
                         'RECOVERY_LANDING','RECOVERY_LANDED'):
            if self.task:
                self.status_state='TASK_ABORTED_TIMEOUT'
                self.publish_status(msg.reason or 'Mission timeout recovery active')
                self._release_lease(); self.task=None; self.targets=[]; self.current_repeat=0; self.repeat_due=0.0; self.last_mission=''
            return
        if msg.current_cell_id:
            target = next((c for c in self.targets if c.cell_id == msg.current_cell_id), None)
            if target:
                self.current_pub.publish(CellTarget(header=Header(stamp=rospy.Time.now(),frame_id='map'),cage_id=target.cage_id,row=target.row,col=target.col,cell_id=target.cell_id,x=target.x,y=target.y,z=target.z,lat=target.lat,lon=target.lon,relative_alt=target.relative_alt))
            else:
                self.current_pub.publish(CellTarget(cell_id=msg.current_cell_id))
        if self.status_state=='TASK_EXECUTING' and msg.mission_id==self.last_mission and msg.state=='FINISHED':
            if self.current_repeat < self.task.repeat_count:
                self.current_repeat+=1; self.status_state='TASK_REPEAT_WAIT'; self.repeat_due=time.monotonic()+self.interval; self.publish_status('Repeat complete; holding before next replan')
            else:
                self.status_state='TASK_COMPLETED'; self.publish_status('All repeats completed; final RTL handled by mission FSM'); self._release_lease(); self.task=None; self.task_map_model=None
                self.update_model()
        if msg.state=='MANUAL_OVERRIDE' and self.status_state in ('TASK_EXECUTING','TASK_REPEAT_WAIT'):
            self.status_state='TASK_MANUAL_OVERRIDE'; self.publish_status('MANUAL CONTROL ACTIVE; no automatic restart'); self._release_lease()
        elif msg.state in ('FAILSAFE','ERROR') and self.status_state in ('TASK_EXECUTING','TASK_REPEAT_WAIT'):
            self.status_state='TASK_ERROR'; self.publish_status(msg.reason or msg.state)
            self._release_lease(); self.task=None; self.targets=[]; self.task_map_model=None
        elif self.status_state in ('TASK_EXECUTING','TASK_REPEAT_WAIT'):
            self.publish_status()

    def tick(self,_):
        if (self.map_lease_id and time.monotonic()-self.last_lease_renew >= self.lease_ttl/2.0
                and not self._renew_lease()):
            self.status_state='TASK_ERROR'; self.publish_status('MAP_LEASE_EXPIRED')
            if self.last_mission: self.mission_cancel_pub.publish(String(data=self.last_mission))
            self._release_lease(); self.task=None; self.targets=[]; self.task_map_model=None
            return
        if self.status_state=='TASK_REPEAT_WAIT' and time.monotonic()>=self.repeat_due and self.task:
            if self.beacon_mode and not rospy.get_param('/realflight/start_ready',False):
                self.status_state='TASK_ERROR'; self.publish_status('REALFLIGHT_PRECHECK_FAILED_BEFORE_REPEAT'); self.task=None; return
            try:
                if self.geometry_mode == 'rectangular':
                    target_model=self.task_map_model if self.use_persistent_map else self.model
                    self.targets=generate_targets(target_model,self.task,(target_model.local.to_enu(self.gps.latitude,self.gps.longitude,self.gps.altitude)[:2]))
                else:
                    result=self.valid_ready(self.task)
                    if isinstance(result,str): raise ValueError(result)
                    self.targets=[]
                self.start_round()
            except Exception as exc:
                self.status_state='TASK_ERROR'; self.publish_status(str(exc))

    def publish_targets(self):
        self.batch_seq += 1
        model=self.task_map_model
        msg = CellTargetArray(header=Header(seq=self.batch_seq,stamp=rospy.Time.now(),frame_id='map'),task_id=self.task.task_id if self.task else '',cage_id=self.active_cage_id(),
            map_uuid=model.map_uuid if model else '',map_version=model.map_version if model else 0,
            map_sha256=model.map_sha256 if model else '',manifest_sha256=model.manifest_sha256 if model else '')
        for c in self.targets:
            msg.targets.append(CellTarget(header=Header(seq=self.batch_seq,stamp=rospy.Time.now(),frame_id='map'),cage_id=c.cage_id,row=c.row,col=c.col,cell_id=c.cell_id,x=c.x,y=c.y,z=c.z,lat=c.lat,lon=c.lon,relative_alt=c.relative_alt))
        self.target_pub.publish(msg)

    def active_cage_id(self):
        if self.geometry_mode == 'circular' and self.geometry is not None:
            return self.geometry.cage_id
        if self.use_persistent_map and self.latest_persistent_model is not None:
            return self.latest_persistent_model.cage_id
        if self.model is not None:
            return self.model.cage_id
        return int(rospy.get_param('/feeding/active_cage_id',0))

    def publish_status(self,message=''):
        req=self.task
        completed = 0
        if self.current_cell_id:
            ids = [c.cell_id for c in self.targets]
            completed = ids.index(self.current_cell_id) + 1 if self.current_cell_id in ids else 0
        cage_id = self.active_cage_id()
        mode = 'CENTER_FEED' if self.geometry_mode == 'circular' and req else (req.mode if req else '')
        total_targets = 1 if self.geometry_mode == 'circular' and req else len(self.targets)
        model=self.task_map_model
        self.status_pub.publish(FeedingTaskStatus(header=Header(stamp=rospy.Time.now()),task_id=req.task_id if req else '',task_state=self.status_state,mode=mode,cage_id=cage_id,current_repeat=self.current_repeat,total_repeat=req.repeat_count if req else 0,completed_targets=completed,total_targets=total_targets,current_cell_id=self.current_cell_id,error_code=self.status_state if self.status_state=='TASK_ERROR' else '',message=message,
            task_map_uuid=model.map_uuid if model else '',task_map_version=model.map_version if model else 0,
            task_map_sha256=model.map_sha256 if model else '',task_manifest_sha256=model.manifest_sha256 if model else ''))


if __name__=='__main__':
    rospy.init_node('feeding_task_manager_node'); FeedingTaskManager(); rospy.spin()
