"""Shared SITL flight lifecycle for startup-selected missions."""
from dataclasses import dataclass
from enum import Enum
from math import isfinite

from ..altitude_control import AltitudeControl

MIN_HEIGHT_M = 0.1
MAX_HEIGHT_M = 100.0
PREARM_SETTLE_S = 12.0
ARM_TIMEOUT_S = 15.0
DISARM_TIMEOUT_S = 10.0
LANDING_DWELL_S = 1.0
HOVER_SPEED_M_S = 0.5
HOVER_ENTRY_DWELL_S = 0.5
LANDING_SPEED_M_S = 0.4
LANDING_HEIGHT_M = 0.2
LOW_HEIGHT_TAKEOFF_THRESHOLD_M = 1.0
MIN_PHASE_TIMEOUT_S = 60.0
CLIMB_TIMEOUT_SPEED_M_S = 0.5
LAND_TIMEOUT_SPEED_M_S = 0.25
PHASE_TIMEOUT_MARGIN_S = 20.0


class Phase(str, Enum):
    PREARM = 'prearm'
    DISARMED = 'disarmed'
    ARMING = 'arming'
    ARMED = 'armed'
    TAKEOFF = 'takeoff'
    HOVER = 'hover'
    LAND = 'land'
    DISARMING = 'disarming'
    FAULT = 'fault'
    SEARCH = 'search'
    CENTER = 'center'
    SETTLE = 'settle'
    LOS = 'los'
    COMMIT = 'commit'


PHASE_IDS = {phase: index for index, phase in enumerate(Phase)}


@dataclass(frozen=True, slots=True)
class Command:
    throttle: int
    arm: bool
    roll: int = 1500
    pitch: int = 1500
    yaw: int = 1500


class MissionBase:
    """Share arm, climb, hover, landing, and disarm transitions."""

    def __init__(self, mode: str, mission_kind: str, height_m: float, hover_duration_s: float):
        if mode not in ('auto', 'manual'):
            raise ValueError('mode must be auto or manual')
        self.mode = mode
        self.phase = Phase.PREARM
        self.entered_at: float | None = None
        self.ground_altitude_m: float | None = None
        self.height_m = height_m
        self.hover_duration_s = hover_duration_s
        self.last_altitude_at = 0.0
        self.landing_dwell_at: float | None = None
        self.near_hover_at: float | None = None
        self.controller = AltitudeControl()
        self.fault_reason: str | None = None
        self.mission_kind = mission_kind
        self.result: str | None = None
        self.started = False

    def _enter(self, phase: Phase, now: float) -> None:
        if phase == self.phase:
            return
        previous = self.phase
        self.phase = phase
        self.entered_at = now
        if phase == Phase.LAND or (phase == Phase.TAKEOFF and previous != Phase.HOVER):
            self.controller.reset()
            self.last_altitude_at = 0.0
        if phase == Phase.TAKEOFF:
            self.near_hover_at = None
        if phase == Phase.LAND:
            self.landing_dwell_at = None

    @property
    def readiness(self) -> int:
        if self.phase == Phase.DISARMED:
            return 1
        if self.phase == Phase.ARMED:
            return 2
        if self.phase in (Phase.TAKEOFF, Phase.HOVER, Phase.LAND, Phase.SEARCH, Phase.CENTER, Phase.SETTLE, Phase.LOS, Phase.COMMIT):
            return 3
        if self.phase == Phase.FAULT:
            return 4
        return 0

    def fault(self, reason: str, now: float) -> None:
        self.fault_reason = reason
        self._enter(Phase.FAULT, now)

    def request(self, action: str, now: float, altitude_m: float | None = None,
                height_m: float | None = None, *, automatic: bool = False) -> None:
        if self.phase == Phase.FAULT:
            raise ValueError('App fault; restart required')
        if self.mode == 'auto' and self.started and action in ('arm', 'start'):
            raise ValueError('Automatic mission permits one attempt per process')
        if action == 'height':
            raise ValueError('Height is fixed by the mission profile')
        elif action == 'arm':
            if automatic != (self.mode == 'auto'):
                raise ValueError('Arm requires the selected approval mode')
            if self.phase != Phase.DISARMED:
                raise ValueError('App is not disarmed and ready')
            self._enter(Phase.ARMING, now)
        elif action == 'start':
            if automatic != (self.mode == 'auto'):
                raise ValueError('Start requires the selected approval mode')
            if self.phase != Phase.ARMED or altitude_m is None or not isfinite(altitude_m):
                raise ValueError('Confirmed arm and fresh altitude required')
            self.ground_altitude_m = altitude_m
            self.started = True
            self._enter(Phase.TAKEOFF, now)
        elif action == 'land':
            if self.phase not in (Phase.TAKEOFF, Phase.HOVER, Phase.SEARCH, Phase.CENTER,
                                  Phase.SETTLE, Phase.LOS, Phase.COMMIT):
                raise ValueError('Active flight required to land')
            self._enter(Phase.LAND, now)
        elif action == 'disarm':
            if self.phase == Phase.ARMED:
                self._enter(Phase.DISARMING, now)
            elif self.phase != Phase.DISARMED:
                raise ValueError('Emergency confirmation required while flying')
        elif action == 'emergency':
            self.fault('Emergency disarm requested', now)
        else:
            raise ValueError('Unsupported flight action')

    def step(self, now: float, altitude_m: float, altitude_at: float,
             firmware_armed: bool, view_alive: bool) -> Command:
        if not isfinite(altitude_m) or altitude_at <= 0 or altitude_at > now or now - altitude_at > 1.0:
            raise ValueError('Fresh finite MAVLink altitude required')
        if self.entered_at is None:
            self.entered_at = now
        if self.mode == 'manual' and not view_alive:
            if self.phase in (Phase.ARMING, Phase.ARMED):
                self._enter(Phase.DISARMING, now)
            elif self.phase in (Phase.TAKEOFF, Phase.HOVER, Phase.SEARCH, Phase.CENTER, Phase.SETTLE, Phase.LOS, Phase.COMMIT):
                self._enter(Phase.LAND, now)
        elapsed = now - self.entered_at
        if self.phase == Phase.PREARM:
            if firmware_armed:
                self.fault('Vehicle armed before App command', now)
            elif elapsed >= PREARM_SETTLE_S:
                self._enter(Phase.DISARMED, now)
        elif self.phase == Phase.ARMING:
            if firmware_armed:
                self._enter(Phase.ARMED, now)
            elif elapsed > ARM_TIMEOUT_S:
                self.fault('Arming not confirmed', now)
        elif self.phase == Phase.ARMED:
            if not firmware_armed:
                self.fault('Vehicle unexpectedly disarmed', now)
        elif self.phase == Phase.DISARMED:
            if firmware_armed:
                self.fault('Vehicle armed without App command', now)
        elif self.phase in (Phase.TAKEOFF, Phase.HOVER, Phase.LAND):
            if not firmware_armed:
                self.fault('Vehicle unexpectedly disarmed', now)
            else:
                target = self.ground_altitude_m + (0.0 if self.phase == Phase.LAND else self.height_m)
                if altitude_at > self.last_altitude_at:
                    self.controller.update(target, altitude_m, altitude_at,
                                           landing=self.phase == Phase.LAND,
                                           low_height_takeoff=(self.phase == Phase.TAKEOFF
                                                               and self.height_m < LOW_HEIGHT_TAKEOFF_THRESHOLD_M))
                    self.last_altitude_at = altitude_at
                speed = self.controller.speed
                hover_band = min(0.5, max(0.03, self.height_m * 0.25))
                near_hover = (self.controller.speed_ready
                              and abs(altitude_m - (self.ground_altitude_m + self.height_m)) <= hover_band
                              and abs(speed) <= HOVER_SPEED_M_S)
                if self.phase == Phase.TAKEOFF:
                    if near_hover:
                        if self.near_hover_at is None:
                            self.near_hover_at = now
                        elif now - self.near_hover_at >= HOVER_ENTRY_DWELL_S:
                            self._enter(Phase.HOVER, now)
                    else:
                        self.near_hover_at = None
                elif self.phase == Phase.HOVER:
                    if not near_hover:
                        self._enter(Phase.TAKEOFF, now)
                    elif elapsed >= self.hover_duration_s:
                        self._on_hover_complete(now)
                elif self.phase == Phase.LAND:
                    landed = (self.controller.speed_ready
                              and abs(altitude_m - self.ground_altitude_m) <= LANDING_HEIGHT_M
                              and abs(speed) <= LANDING_SPEED_M_S)
                    if landed:
                        if self.landing_dwell_at is None:
                            self.landing_dwell_at = now
                        elif now - self.landing_dwell_at >= LANDING_DWELL_S:
                            self._enter(Phase.DISARMING, now)
                    else:
                        self.landing_dwell_at = None
                if self.phase == Phase.TAKEOFF:
                    limit = max(MIN_PHASE_TIMEOUT_S, PHASE_TIMEOUT_MARGIN_S
                                + self.height_m / CLIMB_TIMEOUT_SPEED_M_S)
                elif self.phase == Phase.LAND:
                    limit = max(MIN_PHASE_TIMEOUT_S, PHASE_TIMEOUT_MARGIN_S
                                + self.height_m / LAND_TIMEOUT_SPEED_M_S)
                else:
                    limit = None
                if limit is not None and now - self.entered_at > limit:
                    self.fault(f'{self.phase.value} timed out', now)
        elif self.phase == Phase.DISARMING:
            if not firmware_armed:
                self._enter(Phase.DISARMED, now)
                self.controller.reset()
            elif elapsed > DISARM_TIMEOUT_S:
                self.fault('Disarm not confirmed', now)
        if self.phase in (Phase.TAKEOFF, Phase.HOVER, Phase.LAND, Phase.SEARCH,
                          Phase.CENTER, Phase.SETTLE, Phase.LOS, Phase.COMMIT):
            return Command(self.controller.throttle, True)
        return Command(1000, self.phase in (Phase.ARMING, Phase.ARMED))

    def _on_hover_complete(self, now: float) -> None:
        raise NotImplementedError

    def vision_alive(self, now: float) -> bool:
        return True
