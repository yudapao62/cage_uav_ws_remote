#!/usr/bin/env python3
"""Drive the real feeder through MAVROS RC channel 9 override."""
import math
import threading
import time

import rospy
from cage_feeder.rc9_logic import FeederCycle, rc_override_values, rc_release_value
from cage_msgs.msg import FeederCommand
from cage_msgs.srv import RunFeeder, RunFeederResponse
from mavros_msgs.msg import OverrideRCIn, State
from std_msgs.msg import Bool


class FeederRC9Node:
    def __init__(self):
        prefix = '/feeding/'
        self.channel = int(rospy.get_param(prefix + 'rc_channel', 9))
        self.pwm_on = int(rospy.get_param(prefix + 'rc_pwm_on', 2000))
        self.pwm_off = int(rospy.get_param(prefix + 'rc_pwm_off', 1000))
        self.publish_rate = float(rospy.get_param(prefix + 'rc_publish_rate_hz', 20.0))
        self.status_rate = float(rospy.get_param(prefix + 'status_publish_rate_hz', 5.0))
        self.close_hold_sec = float(rospy.get_param(prefix + 'rc_close_hold_sec', 1.0))
        self.release_frames = int(rospy.get_param(prefix + 'rc_release_frames', 5))
        self.max_duration_sec = float(rospy.get_param(prefix + 'max_run_duration_sec', 300.0))
        self.require_fcu = bool(rospy.get_param(prefix + 'require_fcu_connection', True))
        self.release_value = int(rospy.get_param(prefix + 'rc_extended_release_value', 65534))
        self._validate_config()

        self.lock = threading.RLock()
        self.cycle = FeederCycle(self.close_hold_sec)
        self.fcu_connected = False
        self.last_status = None
        self.last_status_publish = 0.0
        self.release_remaining = 0

        self.rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.status_pub = rospy.Publisher('/feeder/status', Bool, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/feeder/state', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/mavros/state', State, self.on_fcu_state, queue_size=1)
        rospy.Subscriber('/feeder/command', FeederCommand, self.on_command, queue_size=5)
        self.service = rospy.Service('/feeder/run_for_duration', RunFeeder, self.on_run)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.tick)
        rospy.on_shutdown(self.on_shutdown)
        self.publish_status(False, force=True)
        rospy.loginfo(
            'RC9 feeder ready: channel=%d on=%d off=%d max_duration=%.1fs',
            self.channel, self.pwm_on, self.pwm_off, self.max_duration_sec)

    def _validate_config(self):
        if not 1 <= self.channel <= 18:
            raise ValueError('feeding/rc_channel must be in [1, 18]')
        if not 800 <= self.pwm_off <= 2200 or not 800 <= self.pwm_on <= 2200:
            raise ValueError('feeder PWM values must be in [800, 2200]')
        if self.pwm_on == self.pwm_off:
            raise ValueError('feeder ON and OFF PWM values must differ')
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            raise ValueError('feeding/rc_publish_rate_hz must be positive')
        if not math.isfinite(self.status_rate) or self.status_rate <= 0.0:
            raise ValueError('feeding/status_publish_rate_hz must be positive')
        if not math.isfinite(self.close_hold_sec) or self.close_hold_sec <= 0.0:
            raise ValueError('feeding/rc_close_hold_sec must be positive')
        if self.release_frames <= 0:
            raise ValueError('feeding/rc_release_frames must be positive')
        if not math.isfinite(self.max_duration_sec) or self.max_duration_sec <= 0.0:
            raise ValueError('feeding/max_run_duration_sec must be positive')
        if not 0 <= self.release_value <= 65535:
            raise ValueError('feeding/rc_extended_release_value must be in [0, 65535]')

    def on_fcu_state(self, message):
        with self.lock:
            was_connected = self.fcu_connected
            self.fcu_connected = bool(message.connected)
            if not was_connected and self.fcu_connected and self.cycle.state == FeederCycle.RELEASED:
                # Establish a known OFF state once after every MAVROS reconnect.
                self.cycle.begin_closing(time.monotonic())
            if was_connected and not self.fcu_connected and self.cycle.enabled:
                self.cycle.begin_closing(time.monotonic())
                self.publish_status(False)
                rospy.logerr('FCU disconnected while feeder was ON; command cancelled')

    def on_run(self, request):
        duration = float(request.duration_sec)
        if not math.isfinite(duration) or duration <= 0.0 or duration > self.max_duration_sec:
            return RunFeederResponse(
                success=False,
                message='duration_sec must be > 0 and <= {:.1f}'.format(self.max_duration_sec))
        with self.lock:
            if self.require_fcu and not self.fcu_connected:
                return RunFeederResponse(success=False, message='FCU is not connected')
            self.cycle.start_timed(duration, time.monotonic())
            self.release_remaining = 0
            self.publish_status(True)
        rospy.logwarn('Feeder ON for %.3f seconds (RC%d=%d)',
                      duration, self.channel, self.pwm_on)
        return RunFeederResponse(
            success=True,
            message='feeder ON; automatic OFF in {:.3f} seconds'.format(duration))

    def on_command(self, message):
        with self.lock:
            if message.enable and self.require_fcu and not self.fcu_connected:
                self.cycle.begin_closing(time.monotonic())
                self.publish_status(False)
                rospy.logerr('Feeder ON rejected: FCU is not connected (%s)', message.reason)
                return
            self.cycle.set_enabled(bool(message.enable), time.monotonic())
            self.release_remaining = 0
            self.publish_status(bool(message.enable))
        rospy.loginfo('Feeder %s from mission command: %s',
                      'ON' if message.enable else 'OFF', message.reason)

    def override_message(self, value):
        message = OverrideRCIn()
        message.channels = rc_override_values(
            self.channel, value, OverrideRCIn.CHAN_NOCHANGE)
        return message

    def release_message(self):
        value = rc_release_value(
            self.channel, self.release_value, OverrideRCIn.CHAN_RELEASE)
        return self.override_message(value)

    def publish_status(self, enabled, force=False):
        enabled = bool(enabled)
        if force or enabled != self.last_status:
            self.last_status = enabled
            self.last_status_publish = time.monotonic()
            self.status_pub.publish(Bool(data=enabled))
            self.state_pub.publish(Bool(data=enabled))

    def tick(self, _event):
        with self.lock:
            now = time.monotonic()
            transition = self.cycle.update(now)
            state = self.cycle.state
            connected = self.fcu_connected
            if transition and transition[1] == FeederCycle.CLOSING:
                self.publish_status(False)
                rospy.logwarn('Feeder timed run complete; RC%d=%d',
                              self.channel, self.pwm_off)
            if transition and transition[1] == FeederCycle.RELEASED:
                self.release_remaining = self.release_frames
            if now - self.last_status_publish >= 1.0 / self.status_rate:
                self.publish_status(state == FeederCycle.ON, force=True)
            if not connected:
                return
            if state == FeederCycle.ON:
                self.rc_pub.publish(self.override_message(self.pwm_on))
            elif state == FeederCycle.CLOSING:
                self.rc_pub.publish(self.override_message(self.pwm_off))
            elif state == FeederCycle.RELEASED and self.release_remaining > 0:
                # MAVLink channels 9-18 use UINT16_MAX-1 to release override.
                self.rc_pub.publish(self.release_message())
                self.release_remaining -= 1
                if self.release_remaining == 0:
                    rospy.loginfo('RC%d override released to H12', self.channel)

    def on_shutdown(self):
        # Best effort: leave an explicit OFF override; FCU timeout later returns
        # ownership to H12 if this process cannot finish a normal close cycle.
        try:
            if not self.fcu_connected:
                return
            for _ in range(3):
                self.rc_pub.publish(self.override_message(self.pwm_off))
        except (rospy.ROSException, rospy.ROSInterruptException):
            pass


if __name__ == '__main__':
    rospy.init_node('feeder_rc9_node')
    FeederRC9Node()
    rospy.spin()
