"""ROS-independent timing state machine for the RC9 feeder driver."""
import math


def rc_override_values(channel, value, nochange=65535):
    """Return an 18-channel override changing exactly one 1-based channel."""
    if not 1 <= int(channel) <= 18:
        raise ValueError('channel must be in [1, 18]')
    values = [int(nochange)] * 18
    values[int(channel) - 1] = int(value)
    return values


def rc_release_value(channel, extended_release=65534, legacy_release=0):
    """MAVLink uses different release sentinels above channel 8."""
    if not 1 <= int(channel) <= 18:
        raise ValueError('channel must be in [1, 18]')
    return int(legacy_release if int(channel) <= 8 else extended_release)


class FeederCycle:
    RELEASED = 'RELEASED'
    ON = 'ON'
    CLOSING = 'CLOSING'

    def __init__(self, close_hold_sec):
        if not math.isfinite(close_hold_sec) or close_hold_sec <= 0.0:
            raise ValueError('close_hold_sec must be positive')
        self.close_hold_sec = float(close_hold_sec)
        self.state = self.RELEASED
        self.deadline = None
        self.close_deadline = None

    @property
    def enabled(self):
        return self.state == self.ON

    def start_timed(self, duration_sec, now):
        if not math.isfinite(duration_sec) or duration_sec <= 0.0:
            raise ValueError('duration_sec must be positive')
        self.state = self.ON
        self.deadline = float(now) + float(duration_sec)
        self.close_deadline = None

    def set_enabled(self, enabled, now):
        if enabled:
            self.state = self.ON
            self.deadline = None
            self.close_deadline = None
        else:
            self.begin_closing(now)

    def begin_closing(self, now):
        self.state = self.CLOSING
        self.deadline = None
        self.close_deadline = float(now) + self.close_hold_sec

    def update(self, now):
        """Advance timers and return the state transition, if any."""
        previous = self.state
        now = float(now)
        if self.state == self.ON and self.deadline is not None and now >= self.deadline:
            self.begin_closing(now)
        if self.state == self.CLOSING and now >= self.close_deadline:
            self.state = self.RELEASED
            self.close_deadline = None
        if self.state == previous:
            return None
        return previous, self.state
