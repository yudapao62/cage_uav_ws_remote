#!/usr/bin/env python3
import unittest
from cage_mission_fsm.core import MissionFSM, States

class FSMTest(unittest.TestCase):
    def active_fsm(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid(); f.request('m1'); f.planned(); f.upload_result(True); f.armed_and_auto(); return f
    def test_normal_progress(self):
        f=self.active_fsm(); f.waypoint_reached(3,'FEED_START'); self.assertTrue(f.feeder_enabled)
        f.waypoint_reached(4,'FEED_END'); self.assertFalse(f.feeder_enabled)
        f.waypoint_reached(5,'CAGE_EXIT'); self.assertEqual(f.state, States.RTL)
        f.landed(); f.landed(); self.assertEqual(f.state, States.FINISHED)
    def test_manual_override_is_latched_and_feeder_off(self):
        f=self.active_fsm(); f.waypoint_reached(3,'FEED_START'); self.assertTrue(f.mode_changed('LOITER'))
        self.assertEqual(f.state, States.MANUAL_OVERRIDE); self.assertFalse(f.feeder_enabled)
        self.assertFalse(f.request('m2', resume=False)); self.assertEqual(f.state, States.MANUAL_OVERRIDE)
        self.assertFalse(f.request('m2', resume=True)); self.assertEqual(f.state, States.MANUAL_OVERRIDE)
        self.assertTrue(f.auto_reacquired()); self.assertEqual(f.state, States.IDLE)
        self.assertTrue(f.request('m2')); self.assertEqual(f.state, States.PLAN)
    def test_manual_request_preempts_wait_arm_before_auto_confirmation(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid(); f.request('m1'); f.planned(); f.upload_result(True)
        self.assertEqual(f.state, States.WAIT_ARM)
        self.assertFalse(f.mode_changed('STABILIZE', 'AUTO'))
        self.assertEqual(f.state, States.WAIT_ARM)
        self.assertTrue(f.manual_takeover()); self.assertEqual(f.state, States.MANUAL_OVERRIDE)
        self.assertFalse(f.request('m2', resume=True))
    def test_explicit_manual_request_preempts_every_automatic_stage(self):
        automatic_stages = (
            States.PLAN, States.UPLOAD_MISSION, States.WAIT_ARM, States.TAKEOFF,
            States.TRANSIT, States.FEEDING, States.FEEDING_DWELL, States.EXIT,
            States.RTL, States.LAND, States.TIMEOUT_ABORT, States.RECOVERY_RTL,
            States.RECOVERY_RTL_VERIFY, States.RECOVERY_RETURNING,
            States.RECOVERY_LANDING,
        )
        for stage in automatic_stages:
            f = MissionFSM(); f.transition(stage); f.feeder_enabled = True
            self.assertTrue(f.manual_takeover(), stage)
            self.assertEqual(f.state, States.MANUAL_OVERRIDE, stage)
            self.assertFalse(f.feeder_enabled, stage)
    def test_disconnect_failsafe_turns_feeder_off(self):
        f=self.active_fsm(); f.waypoint_reached(3,'FEED_START'); f.failsafe('FCU_DISCONNECTED')
        self.assertEqual(f.state, States.FAILSAFE); self.assertFalse(f.feeder_enabled)
    def test_grid_chain_can_hold_without_rtl_for_repeat(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid(); f.request('m1', rtl_after_mission=False); f.planned(); f.upload_result(True); f.armed_and_auto()
        f.waypoint_reached(2,'GRID_ENTER'); f.waypoint_reached(3,'CELL_TARGET','R01C01')
        self.assertEqual(f.current_cell_id, 'R01C01'); self.assertTrue(f.feeder_enabled)
        f.waypoint_reached(4,'GRID_EXIT')
        self.assertEqual(f.state, States.FINISHED); self.assertFalse(f.feeder_enabled)
    def test_feeder_is_explicitly_disabled_between_cell_targets(self):
        f=self.active_fsm(); f.waypoint_reached(3,'CELL_TARGET','R01C01')
        self.assertTrue(f.feeder_enabled); self.assertEqual(f.state,States.FEEDING)
        self.assertTrue(f.feeder_departed_cell())
        self.assertFalse(f.feeder_enabled); self.assertEqual(f.state,States.FEEDING)
        self.assertFalse(f.feeder_departed_cell())
        f.waypoint_reached(4,'CELL_TARGET','R01C02')
        self.assertTrue(f.feeder_enabled); self.assertEqual(f.current_cell_id,'R01C02')
    def test_timeout_recovery_chain(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid(); f.request('m1'); f.planned(); f.upload_result(True); f.armed_and_auto()
        f.waypoint_reached(3,'FEED_START'); f.timeout_abort('FEEDING_TIMEOUT')
        self.assertEqual(f.state, States.TIMEOUT_ABORT); self.assertTrue(f.mission_aborted); self.assertTrue(f.recovery_active)
        f.recovery_start(); self.assertEqual(f.state, States.RECOVERY_RTL)
        f.transition(States.RECOVERY_RTL_VERIFY, 'RTL_REQUESTED'); f.recovery_rtl_verified(); self.assertEqual(f.state, States.RECOVERY_RETURNING)
        f.recovery_landing(); self.assertEqual(f.state, States.RECOVERY_LANDING)
        f.recovery_landed(); self.assertEqual(f.state, States.FINISHED); self.assertFalse(f.recovery_active)
    def test_circular_dwell_then_rtl(self):
        f=self.active_fsm(); self.assertTrue(f.feed_point_arrived(2))
        self.assertEqual(f.state,States.FEEDING_DWELL); self.assertTrue(f.feeder_enabled)
        self.assertTrue(f.feeding_dwell_complete())
        self.assertEqual(f.state,States.RTL); self.assertFalse(f.feeder_enabled)
    def test_manual_takeover_during_circular_dwell(self):
        f=self.active_fsm(); f.feed_point_arrived(2); self.assertTrue(f.mode_changed('LOITER'))
        self.assertEqual(f.state,States.MANUAL_OVERRIDE); self.assertFalse(f.feeder_enabled)
    def test_anchor_failure_during_circular_dwell(self):
        f=self.active_fsm(); f.feed_point_arrived(2); f.timeout_abort('BEACON_LOST')
        self.assertEqual(f.state,States.TIMEOUT_ABORT); self.assertFalse(f.feeder_enabled)
    def test_scout_waits_for_map_finalize_before_rtl(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid()
        self.assertTrue(f.request('scout-1', mission_type=1, auto_land=True))
        f.planned(); f.upload_result(True); f.armed_and_auto()
        f.waypoint_reached(0,'TAKEOFF'); f.waypoint_reached(1,'TRANSIT')
        f.waypoint_reached(2,'SCOUT_ENTER')
        self.assertEqual(f.state,States.SCOUTING); self.assertFalse(f.feeder_enabled)
        f.waypoint_reached(3,'SCOUT_PASS'); f.waypoint_reached(4,'SCOUT_EXIT')
        self.assertEqual(f.state,States.FINALIZING_MAP)
        self.assertTrue(f.scout_finalized(True)); self.assertEqual(f.state,States.RTL)
        f.landed(); self.assertEqual(f.state,States.LAND)
        f.landed(); self.assertEqual(f.state,States.FINISHED)
    def test_scout_finalize_failure_still_enters_rtl(self):
        f=MissionFSM(); f.state=States.FINALIZING_MAP; f.mission_type=1
        self.assertTrue(f.scout_finalized(False,'MAP_WRITE_FAILED'))
        self.assertEqual(f.state,States.RTL); self.assertTrue(f.mission_aborted)
        self.assertFalse(f.feeder_enabled)
    def test_cancel_in_air_uses_recovery(self):
        f=MissionFSM(); f.state=States.SCOUTING; f.auto_control_started=True
        self.assertTrue(f.cancel()); self.assertEqual(f.state,States.TIMEOUT_ABORT)
        self.assertTrue(f.recovery_active); self.assertFalse(f.feeder_enabled)

if __name__ == '__main__': unittest.main()
