"""Measured yaw sweep, independent of command ticks and wall-clock arrival."""
from math import degrees, pi, radians

from los_tracking.geometry import wrap


SEARCH_RATE_RAD_S = radians(15)
SEARCH_TIMEOUT_S = 34.0
SEARCH_CIRCLE_RAD = 2 * pi
YAW_SETTLE_DURATION_S = 0.3
YAW_SETTLE_RATE_RAD_S = radians(5)
YAW_SETTLE_ERROR_RAD = radians(2)
ANGLE_EPSILON_RAD = 1e-9


class YawSearch:
    def __init__(self):
        self.yaw_rad = self.last_sim_s = None
        self.unwrapped_rad = self.yaw_rate_rad_s = 0.0
        self.started_sim = self.start_angle_rad = self.settled_sim = None

    def observe(self, yaw_rad, simulation_time_s):
        if self.last_sim_s is not None:
            if simulation_time_s <= self.last_sim_s:
                return  # Repeated feedback is not another rotation measurement.
            delta = wrap(yaw_rad - self.yaw_rad)
            self.unwrapped_rad += delta
            self.yaw_rate_rad_s = delta / (simulation_time_s - self.last_sim_s)
        self.yaw_rad, self.last_sim_s = yaw_rad, simulation_time_s

    def start(self, simulation_time_s):
        if self.started_sim is None:
            if self.yaw_rad is None:
                raise ValueError('search needs measured attitude')
            self.started_sim = simulation_time_s
            self.start_angle_rad = self.unwrapped_rad
        self.settled_sim = None

    @property
    def swept_rad(self):
        return 0.0 if self.start_angle_rad is None else max(0.0, self.unwrapped_rad - self.start_angle_rad)

    @property
    def exhausted(self):
        return self.swept_rad >= SEARCH_CIRCLE_RAD - ANGLE_EPSILON_RAD

    def elapsed(self, simulation_time_s):
        return 0.0 if self.started_sim is None else simulation_time_s - self.started_sim

    def diagnostics(self, simulation_time_s, *, rotating):
        return dict(search_swept_deg=degrees(self.swept_rad),
                    search_remaining_deg=degrees(max(0., SEARCH_CIRCLE_RAD - self.swept_rad)),
                    search_commanded_yaw_rate_deg_s=degrees(SEARCH_RATE_RAD_S) if rotating else 0.,
                    search_measured_yaw_rate_deg_s=degrees(self.yaw_rate_rad_s),
                    search_elapsed_s=self.elapsed(simulation_time_s))
