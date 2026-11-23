"""One-attempt MAVLink-and-bbox red-truck mission."""
from math import cos, degrees, isfinite, pi, radians, sin, sqrt
import re
from beta_client.timesync import TimeSyncStatus

from los_tracking import LosNavigator
from los_tracking.config import NavigatorConfig
from los_tracking.types import AttitudeSample, BarometerSample, State, TargetObservation

from ..altitude_control import HOVER_THROTTLE
from ..blackboard import StepContext
from ..sensor_estimator import TargetSensorEstimator
from ..tracking_log import TrackingLog
from .base import Action, Command, MAX_HEIGHT_M, MIN_HEIGHT_M, MissionBase, Mode, Phase, Readiness

TARGET_HEIGHT_M = 20.0
TARGET_HOVER_S = 5.0
SEARCH_RATE_RAD_S = radians(15)
SEARCH_TIMEOUT_S = 60.0
SEARCH_CIRCLE_RAD = 2 * pi
CENTER_TOLERANCE_PX = 10
CENTER_GAIN_S_INV = 1.0
SETTLE_S = 0.3
SETTLE_RATE_RAD_S = radians(5)
APPROACH_TIMEOUT_S = 120.0
COMMIT_TIMEOUT_S = 3.0
MAX_BBOX_AGE_S = 0.2
PREFLIGHT_FEED_AGE_S = 0.5
SITL_STREAM_ID = 'betaflight-sitl'
RC_CENTER = 1500
RC_HALF_RANGE = 500
ANGLE_SCALE_RAD = pi / 4
THROTTLE_MIN = 1050
THROTTLE_MAX = 2000
IDLE_FRACTION = 0.055
TIMESYNC_STARTUP_TIMEOUT_S = 5.0
TARGET_PHASES = frozenset((Phase.SEARCH, Phase.CENTER, Phase.SETTLE, Phase.LOS, Phase.COMMIT))
TARGET_PHASE_EDGES = (
    (Phase.HOVER, Phase.SEARCH),
    (Phase.SEARCH, Phase.CENTER),
    (Phase.SEARCH, Phase.SETTLE),
    (Phase.CENTER, Phase.SEARCH),
    (Phase.CENTER, Phase.SETTLE),
    (Phase.SETTLE, Phase.SEARCH),
    (Phase.SETTLE, Phase.CENTER),
    (Phase.SETTLE, Phase.LOS),
    (Phase.LOS, Phase.COMMIT),
)


def _wrap(angle):
    return (angle + pi) % (2 * pi) - pi


def _quaternion(roll, pitch, yaw):
    """Invert the SITL MAVLink FRD/NED attitude into Bullet FLU/Z-up."""
    cr, sr = cos(roll / 2), sin(roll / 2)
    cp, sp = cos(pitch / 2), sin(pitch / 2)
    cy, sy = cos(yaw / 2), sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, -(cr * sp * cy + sr * cp * sy),
            -(cr * cp * sy - sr * sp * cy), cr * cp * cy + sr * sp * sy)


def _yaw_pwm(rate_rad_s):
    requested = degrees(abs(rate_rad_s))
    stick = (-70 + sqrt(70**2 + 2400 * requested)) / 1200
    return round(RC_CENTER + RC_HALF_RANGE * stick * (1 if rate_rad_s >= 0 else -1))


def _demand_command(demand, config, *, hover_pwm):
    roll = round(RC_CENTER + RC_HALF_RANGE * demand.roll_rad / ANGLE_SCALE_RAD)
    pitch = round(RC_CENTER + RC_HALF_RANGE * demand.pitch_rad / ANGLE_SCALE_RAD)
    def betaflight_throttle(thrust_n):
        fraction = sqrt(min(1.0, max(0.0, thrust_n / config.vehicle.max_thrust_n)))
        stick = min(1.0, max(0.0, (fraction - IDLE_FRACTION) / (1 - IDLE_FRACTION)))
        return round(THROTTLE_MIN + (THROTTLE_MAX - THROTTLE_MIN) * stick)
    nominal_hover = betaflight_throttle(config.vehicle.mass_kg * config.vehicle.gravity_m_s2)
    throttle = betaflight_throttle(demand.thrust_n) + hover_pwm - nominal_hover
    return Command(max(THROTTLE_MIN, min(THROTTLE_MAX, throttle)), True,
                   max(1000, min(2000, roll)), max(1000, min(2000, pitch)),
                   max(1000, min(2000, _yaw_pwm(demand.yaw_rate_rad_s))))


class TargetMission(MissionBase):
    def __init__(self, config_path, mode: Mode | str = Mode.MANUAL, blackboard=None):
        super().__init__(mode, 'target-hit', TARGET_HEIGHT_M, TARGET_HOVER_S, blackboard)
        self.airborne_phases = self.airborne_phases | TARGET_PHASES
        for source in TARGET_PHASES:
            self._add_transition(source, Phase.LAND)
            self._add_transition(source, Phase.DISARMING)
        for source, destination in TARGET_PHASE_EDGES:
            self._add_transition(source, destination)
        self.config = NavigatorConfig.from_yaml(config_path)
        self.navigator = LosNavigator(self.config)
        self.sensor_estimator = TargetSensorEstimator(self.config.tuning)
        self.tracking_log = TrackingLog()
        self.attempted = False
        self.search_started = None
        self.last_yaw = None
        self.swept = 0.0
        self.settled_at = None
        self.los_started = None
        self.los_hover_pwm = HOVER_THROTTLE
        self.commit_started = None
        self.timesync_status: TimeSyncStatus | None = None
        self.timesync_started_at = None
        self.timesync_observed_at = 0.0
        self.last_attitude_boot_ms = None
        self.last_attitude_mapped_age_s = None
        self.attitude_mapping_rejections = 0

    def _on_phase_changed(self, event):
        super()._on_phase_changed(event)
        if self.phase == Phase.SEARCH and event.transition.source == Phase.HOVER.name:
            self.search_started = event.kwargs['now']
            self.swept = 0.0
            self.last_yaw = None
        elif self.phase == Phase.LOS:
            self.los_started = event.kwargs['now']
            self.los_hover_pwm = self.controller.throttle
            self.sensor_estimator.start_los()

    def request_height(self, height_m: float) -> None:
        if self.mode != Mode.MANUAL:
            raise ValueError('Target height requires manual mode')
        if self.phase not in (Phase.DISARMED, Phase.ARMED) or self.started or self.attempted:
            raise ValueError('Target height requires idle flight before Start')
        if type(height_m) not in (int, float) or not isfinite(height_m) or not MIN_HEIGHT_M <= height_m <= MAX_HEIGHT_M:
            raise ValueError('Target height must be 0.1–100 m')
        self.height_m = float(height_m)

    def observe_bbox(self, packet, now):
        fields = {'schema_version', 'image_topic', 'session', 'sequence', 'capture_ns',
                  'width', 'height', 'drone_id', 'bbox'}
        if not isinstance(packet, dict) or set(packet) not in (fields, fields | {'fid'}) or packet['schema_version'] != 3:
            return False
        try:
            stamp = packet['capture_ns'] / 1e9
            if (type(packet['capture_ns']) is not int or not 0 <= now - stamp <= MAX_BBOX_AGE_S
                    or ('fid' in packet and (type(packet['fid']) is not int or not 0 <= packet['fid'] <= 2**53 - 1))
                    or packet['width'] != self.config.camera.width_px
                    or packet['height'] != self.config.camera.height_px
                    or packet['image_topic'] != 'camera/front/image' or packet['drone_id'] != 'drone_1'
                    or not isinstance(packet['session'], str)
                    or re.fullmatch('[0-9a-f]{32}', packet['session']) is None
                    or type(packet['sequence']) is not int or packet['sequence'] <= 0):
                return False
            box = packet['bbox']
            if box is not None:
                if not isinstance(box, dict) or set(box) != {'x', 'y', 'width', 'height'}:
                    return False
                x, y, w, h = (box[key] for key in ('x', 'y', 'width', 'height'))
                if any(type(v) is not int for v in (x, y, w, h)) or min(x, y) < 0 or min(w, h) <= 0 or x + w > packet['width'] or y + h > packet['height']:
                    return False
            self.blackboard.visual.bbox = {**packet, 'bbox': None if box is None else dict(box)}
            if self.phase == Phase.LOS:
                self.sensor_estimator.push_target(TargetObservation(
                    packet['session'], packet['sequence'], stamp,
                    None if box is None else (box['x'], box['y'], box['width'], box['height'])))
            return True
        except (KeyError, TypeError, OverflowError, ZeroDivisionError):
            return False

    def observe_simulation_contact(self, event):
        if event is not None and event.get('target_id') == 'red_truck' and event.get('drone_id') == 'drone_1' and self.phase in (
                Phase.SEARCH, Phase.CENTER, Phase.SETTLE, Phase.LOS, Phase.COMMIT):
            self.blackboard.simulation.target_contact = True

    def request(self, action: Action, now, altitude_m=None, height_m=None, *, automatic=False):
        if action in (Action.ARM, Action.START) and not self.sync_ready(now):
            raise ValueError('MAVLink TIMESYNC not ready')
        if self.attempted and action in (Action.ARM, Action.START):
            raise ValueError('Target mission permits one attempt per process')
        super().request(action, now, altitude_m, height_m, automatic=automatic)
        if action == Action.START:
            self.attempted = True

    @property
    def readiness(self) -> Readiness:
        if self.phase in (Phase.DISARMED, Phase.ARMED) and not self.sync_ready(self.timesync_observed_at):
            return Readiness.UNAVAILABLE
        return super().readiness

    def sync_ready(self, now):
        return self.timesync_status is not None and self.timesync_status.fresh(now)

    def observe_timesync(self, status, now):
        was_ready = self.sync_ready(self.timesync_observed_at)
        if self.timesync_started_at is None:
            self.timesync_started_at = now
        self.timesync_status = status
        self.timesync_observed_at = now
        if was_ready and not self.sync_ready(now):
            self.last_attitude_boot_ms = None
        if self.sync_ready(now) or self.phase in (Phase.FAULT, Phase.DISARMING, Phase.LAND):
            return
        if self.phase in (Phase.PREARM, Phase.DISARMED):
            if now - self.timesync_started_at >= TIMESYNC_STARTUP_TIMEOUT_S:
                self.fault('MAVLink TIMESYNC unavailable', now)
        elif self.phase in (Phase.ARMING, Phase.ARMED):
            self.result = 'MAVLink TIMESYNC lost'
            self._enter(Phase.DISARMING, now)
        else:
            self._fail('MAVLink TIMESYNC lost', now)

    def vision_alive(self, now):
        bbox = self.blackboard.visual.bbox
        return (bbox is not None and 0 <= now - bbox['capture_ns'] / 1e9 <= PREFLIGHT_FEED_AGE_S)

    def _fail(self, reason, now):
        self.result = reason
        self._enter(Phase.LAND, now)

    def _visual_box(self, now):
        bbox = self.blackboard.visual.bbox
        if bbox is None or now - bbox['capture_ns'] / 1e9 > MAX_BBOX_AGE_S:
            return None
        return bbox['bbox']

    def _update_search_yaw(self, sample):
        if sample.attitude is None or sample.attitude_at <= 0:
            return
        yaw = -sample.attitude.yaw
        if self.last_yaw is not None and self.phase in (Phase.SEARCH, Phase.CENTER, Phase.SETTLE):
            self.swept += max(0.0, _wrap(yaw - self.last_yaw))
        self.last_yaw = yaw

    def _observe_telemetry(self, now):
        sample = self.blackboard.telemetry
        if sample.attitude is not None and 0 <= now - sample.attitude_at <= self.config.tuning.max_attitude_age_s:
            a = sample.attitude
            if self.sync_ready(now) and (self.last_attitude_boot_ms is None or a.time_boot_ms > self.last_attitude_boot_ms):
                self.last_attitude_boot_ms = a.time_boot_ms
                measured_at = sample.attitude_at
                self.last_attitude_mapped_age_s = now - measured_at
                if now - measured_at <= self.config.tuning.max_attitude_age_s:
                    self.sensor_estimator.push_attitude(AttitudeSample(
                        SITL_STREAM_ID, a.time_boot_ms, measured_at,
                        _quaternion(a.roll, a.pitch, a.yaw)))
                else:
                    self.attitude_mapping_rejections += 1
        if sample.vfr_hud is not None and 0 <= now - sample.vfr_hud_at <= self.config.tuning.max_barometer_age_s:
            self.sensor_estimator.push_barometer(BarometerSample(
                SITL_STREAM_ID, round(sample.vfr_hud_at * 1e9),
                sample.vfr_hud_at, sample.vfr_hud.alt))

    def _sensor_inputs(self, now):
        return self.sensor_estimator.tick(now)

    def _target_step(self, context: StepContext) -> Command:
        now = context.now_s
        altitude = context.altitude_m
        altitude_at = context.altitude_at_s
        firmware_armed = context.firmware_armed
        view_alive = context.view_alive
        if not firmware_armed:
            self.fault('Vehicle unexpectedly disarmed', now)
            return Command(1000, False)
        if self.mode == Mode.MANUAL and not view_alive:
            self._fail('Bullet View connection lost', now)
            return super().step(context)
        if self.blackboard.simulation.target_contact:
            self.result = 'hit'
            self._enter(Phase.DISARMING, now)
            return Command(1000, False)
        if altitude_at > self.last_altitude_at:
            self.controller.update(self.ground_altitude_m + self.height_m, altitude, altitude_at, landing=False)
            self.last_altitude_at = altitude_at
        box = self._visual_box(now)
        self._update_search_yaw(self.blackboard.telemetry)
        if self.phase in (Phase.SEARCH, Phase.CENTER, Phase.SETTLE):
            if self.search_started is None:
                self.search_started = now
            if now - self.search_started >= SEARCH_TIMEOUT_S or self.swept >= SEARCH_CIRCLE_RAD:
                self._fail('target not found', now)
                return super().step(context)
            if box is None:
                self._enter(Phase.SEARCH, now)
                self.settled_at = None
                yaw_rate = SEARCH_RATE_RAD_S
            else:
                error = box['x'] + box['width'] / 2 - self.config.camera.cx_px
                if abs(error) > CENTER_TOLERANCE_PX:
                    self._enter(Phase.CENTER, now)
                    self.settled_at = None
                    yaw_rate = max(-SEARCH_RATE_RAD_S, min(SEARCH_RATE_RAD_S,
                        -CENTER_GAIN_S_INV * error / self.config.camera.fx_px))
                else:
                    self._enter(Phase.SETTLE, now)
                    yaw_rate = 0.0
                    sample = self.blackboard.telemetry
                    rate = sample.attitude.yawspeed if sample.attitude is not None else float('inf')
                    if abs(rate) <= SETTLE_RATE_RAD_S:
                        if self.settled_at is None:
                            self.settled_at = now
                        elif now - self.settled_at >= SETTLE_S:
                            self._enter(Phase.LOS, now)
                    else:
                        self.settled_at = None
            if self.phase != Phase.LOS:
                return Command(self.controller.throttle, True, yaw=_yaw_pwm(yaw_rate))
        if now - self.los_started >= APPROACH_TIMEOUT_S:
            self._fail('approach timed out', now)
            return super().step(context)
        sample = self.blackboard.telemetry
        if (sample.attitude is None or now - sample.attitude_at > self.config.tuning.max_attitude_age_s):
            self._fail('LOS attitude source stale', now)
            return super().step(context)
        if (sample.vfr_hud is None or now - sample.vfr_hud_at > self.config.tuning.max_barometer_age_s):
            self._fail('LOS altitude source stale', now)
            return super().step(context)
        self.blackboard.features.navigation_input = self._sensor_inputs(now)
        result = self.navigator.update(self.blackboard.features.navigation_input, now_s=now, enabled=True,
                                       command_horizon_s=0.02)
        self.blackboard.features.navigation_result = result
        if result.state == State.FAULT:
            self._fail(f'LOS {result.reason}', now)
            return super().step(context)
        if result.state == State.COMMIT:
            self._enter(Phase.COMMIT, now)
            if self.commit_started is None:
                self.commit_started = now
            elif now - self.commit_started >= COMMIT_TIMEOUT_S:
                self._fail('contact not confirmed', now)
                return super().step(context)
        if result.command is None or now >= result.valid_until_s:
            return Command(self.controller.throttle, True)
        return _demand_command(result.command, self.config, hover_pwm=self.los_hover_pwm)

    def step(self, context: StepContext) -> Command:
        if context.blackboard is not self.blackboard:
            raise ValueError('Mission step uses a different blackboard')
        self._observe_telemetry(context.now_s)
        if self.phase in (Phase.SEARCH, Phase.CENTER, Phase.SETTLE, Phase.LOS, Phase.COMMIT):
            return self._target_step(context)
        return super().step(context)

    def on_command_sent(self, context: StepContext, command: Command) -> None:
        """Record target guidance against the command App actually sent."""
        self.tracking_log.observe(self, context, command)

    def _on_hover_complete(self, context: StepContext) -> None:
        self._enter(Phase.SEARCH, context.now_s)
