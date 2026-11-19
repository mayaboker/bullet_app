"""Bounded, fixed-tick sensor estimates for the target-hit LOS adapter."""
from collections import deque
from math import acos, exp, floor, isfinite

from los_tracking.geometry import slerp
from los_tracking.types import AttitudeSample, BarometerSample, NavigationInput, TargetObservation

ESTIMATE_PERIOD_S = 0.02
ATTITUDE_SMOOTHING_S = 0.04
MAX_PREDICTION_S = 0.06
MAX_ANGULAR_RATE_RAD_S = 4.0
TARGET_QUEUE_CAPACITY = 16
TIME_EPSILON_S = 1e-9
ESTIMATED_STREAM_ID = 'betaflight-sitl-estimated'


class TargetSensorEstimator:
    """Use real measurement times; never renew a camera observation's age."""

    def __init__(self, tuning):
        self.tuning = tuning
        self.attitude_previous = None
        self.attitude_latest = None
        self.smoothed_previous = None
        self.smoothed_latest = None
        self.barometer_previous = None
        self.barometer_latest = None
        self.targets = deque()
        self.last_target_identity = None
        self.next_tick_s = None
        self.first_attitude_tick_s = None
        self.sequence = 0
        self.skipped_ticks = 0
        self.expired_boxes = 0
        self.overflow_boxes = 0
        self.reordered_samples = 0
        self.attitude_interpolations = 0
        self.barometer_interpolations = 0
        self.barometer_missing_ticks = 0
        self.last_tick_s = None
        self.last_attitude_prediction_age_s = None
        self.last_altitude_prediction_age_s = None

    @property
    def queue_depth(self):
        return len(self.targets)

    def start_los(self):
        self.next_tick_s = None
        self.first_attitude_tick_s = None
        self.last_tick_s = None

    def push_attitude(self, sample: AttitudeSample):
        previous = self.attitude_latest
        if previous is not None and (sample.stream_id != previous.stream_id
                                     or sample.sequence <= previous.sequence
                                     or sample.timestamp_s <= previous.timestamp_s):
            if sample != previous:
                self.reordered_samples += 1
            return
        self.attitude_previous = previous
        self.attitude_latest = sample
        if previous is None or self.smoothed_latest is None:
            self.smoothed_previous = None
            self.smoothed_latest = sample.orientation_xyzw
        else:
            self.smoothed_previous = self.smoothed_latest
            dt = sample.timestamp_s - previous.timestamp_s
            alpha = 1 - exp(-dt / ATTITUDE_SMOOTHING_S)
            self.smoothed_latest = slerp(self.smoothed_latest, sample.orientation_xyzw, alpha)

    def push_barometer(self, sample: BarometerSample):
        previous = self.barometer_latest
        if previous is not None and (sample.stream_id != previous.stream_id
                                     or sample.sequence <= previous.sequence
                                     or sample.timestamp_s <= previous.timestamp_s):
            if sample != previous:
                self.reordered_samples += 1
            return
        self.barometer_previous = previous
        self.barometer_latest = sample

    def push_target(self, sample: TargetObservation):
        previous = self.last_target_identity
        if previous is not None and (sample.stream_id == previous.stream_id
                                     and (sample.sequence <= previous.sequence
                                          or sample.timestamp_s <= previous.timestamp_s)):
            if sample != previous:
                self.reordered_samples += 1
            return
        self.last_target_identity = sample
        if len(self.targets) >= TARGET_QUEUE_CAPACITY:
            self.targets.popleft()
            self.overflow_boxes += 1
        self.targets.append(sample)

    def _attitude_at(self, tick_s):
        latest = self.attitude_latest
        if latest is None:
            return None
        previous = self.attitude_previous
        if latest.timestamp_s > tick_s + TIME_EPSILON_S:
            if previous is None or self.smoothed_previous is None:
                return None
            age = tick_s - previous.timestamp_s
            if not -TIME_EPSILON_S <= age <= MAX_PREDICTION_S + TIME_EPSILON_S:
                return None
            fraction = age / (latest.timestamp_s - previous.timestamp_s)
            self.last_attitude_prediction_age_s = max(0.0, age)
            self.attitude_interpolations += 1
            orientation = slerp(self.smoothed_previous, self.smoothed_latest, fraction)
            return AttitudeSample(ESTIMATED_STREAM_ID, self.sequence, tick_s, orientation)
        lead = tick_s - latest.timestamp_s
        if not -TIME_EPSILON_S <= lead <= MAX_PREDICTION_S + TIME_EPSILON_S:
            return None
        self.last_attitude_prediction_age_s = max(0.0, lead)
        orientation = self.smoothed_latest
        if previous is not None and self.smoothed_previous is not None and lead > 0:
            interval = latest.timestamp_s - previous.timestamp_s
            dot = abs(sum(a * b for a, b in zip(self.smoothed_previous, orientation)))
            angle = 2 * acos(min(1.0, dot))
            if not isfinite(angle) or angle / interval > MAX_ANGULAR_RATE_RAD_S:
                return None
            orientation = slerp(self.smoothed_previous, orientation, 1 + lead / interval)
        return AttitudeSample(ESTIMATED_STREAM_ID, self.sequence, tick_s, orientation)

    def _barometer_at(self, tick_s):
        latest = self.barometer_latest
        if latest is None:
            return None
        previous = self.barometer_previous
        if latest.timestamp_s > tick_s + TIME_EPSILON_S:
            if previous is None:
                return None
            age = tick_s - previous.timestamp_s
            if not -TIME_EPSILON_S <= age <= MAX_PREDICTION_S + TIME_EPSILON_S:
                return None
            fraction = age / (latest.timestamp_s - previous.timestamp_s)
            self.last_altitude_prediction_age_s = max(0.0, age)
            self.barometer_interpolations += 1
            altitude = previous.altitude_m + fraction * (latest.altitude_m - previous.altitude_m)
            return BarometerSample(ESTIMATED_STREAM_ID, self.sequence, tick_s, altitude)
        lead = tick_s - latest.timestamp_s
        if not -TIME_EPSILON_S <= lead <= MAX_PREDICTION_S + TIME_EPSILON_S:
            return None
        self.last_altitude_prediction_age_s = max(0.0, lead)
        altitude = latest.altitude_m
        if previous is not None and lead > 0:
            interval = latest.timestamp_s - previous.timestamp_s
            velocity = (latest.altitude_m - previous.altitude_m) / interval
            velocity = max(-self.tuning.max_descent_m_s,
                           min(self.tuning.max_climb_m_s, velocity))
            altitude += velocity * lead
        return BarometerSample(ESTIMATED_STREAM_ID, self.sequence, tick_s, altitude)

    def tick(self, now_s):
        if self.next_tick_s is None:
            self.next_tick_s = now_s
        if now_s + TIME_EPSILON_S < self.next_tick_s:
            return NavigationInput()
        missed = max(0, floor((now_s - self.next_tick_s + TIME_EPSILON_S) / ESTIMATE_PERIOD_S))
        self.skipped_ticks += missed
        tick_s = self.next_tick_s + missed * ESTIMATE_PERIOD_S
        self.next_tick_s = tick_s + ESTIMATE_PERIOD_S
        self.last_tick_s = tick_s
        self.sequence += 1
        self.last_attitude_prediction_age_s = None
        self.last_altitude_prediction_age_s = None
        attitude = self._attitude_at(tick_s)
        barometer = self._barometer_at(tick_s)
        if barometer is None:
            self.barometer_missing_ticks += 1
        if attitude is not None and self.first_attitude_tick_s is None:
            self.first_attitude_tick_s = tick_s
        while self.targets and (now_s - self.targets[0].timestamp_s > self.tuning.max_image_age_s
                                or self.first_attitude_tick_s is not None
                                and self.targets[0].timestamp_s < self.first_attitude_tick_s):
            self.targets.popleft()
            self.expired_boxes += 1
        target = None
        if (attitude is not None and self.first_attitude_tick_s is not None and self.targets
                and self.targets[0].timestamp_s <= tick_s + TIME_EPSILON_S):
            target = self.targets.popleft()
        return NavigationInput(target, barometer, attitude)
