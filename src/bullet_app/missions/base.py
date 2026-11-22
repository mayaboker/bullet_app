"""Shared SITL flight lifecycle for startup-selected missions."""
from dataclasses import dataclass
from enum import Enum, IntEnum
from math import isfinite
from loguru import logger
from transitions import Machine

from ..altitude_control import AltitudeControl
from ..blackboard import Blackboard, StepContext

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


class Readiness(IntEnum):
    """Numeric mission readiness carried in MAVLink MIS_READY."""

    UNAVAILABLE = 0
    DISARMED_IDLE = 1
    ARMED_IDLE = 2
    FLYING = 3
    RESET_REQUIRED = 4


class Action(Enum):
    """App-owned requests that can change a mission's flight state."""

    ARM = 'arm'
    START = 'start'
    LAND = 'land'
    DISARM = 'disarm'
    EMERGENCY = 'emergency'
    HEIGHT = 'height'


class Mode(Enum):
    """Select automatic or operator-approved mission actions."""

    AUTO = 'auto'
    MANUAL = 'manual'


PHASE_IDS = {phase: index for index, phase in enumerate(Phase)}
AIRBORNE_PHASES = frozenset((Phase.TAKEOFF, Phase.HOVER, Phase.LAND))
SHARED_PHASE_EDGES = (
    (Phase.PREARM, Phase.DISARMED),
    (Phase.DISARMED, Phase.ARMING),
    (Phase.ARMING, Phase.ARMED),
    (Phase.ARMED, Phase.TAKEOFF),
    (Phase.TAKEOFF, Phase.HOVER),
    (Phase.HOVER, Phase.TAKEOFF),
    (Phase.LAND, Phase.DISARMING),
    (Phase.DISARMING, Phase.DISARMED),
)


@dataclass(frozen=True, slots=True)
class Command:
    throttle: int
    arm: bool
    roll: int = 1500
    pitch: int = 1500
    yaw: int = 1500


class MissionBase:
    """Share arm, climb, hover, landing, and disarm transitions."""

    def __init__(self, mode: Mode | str, mission_kind: str, height_m: float, hover_duration_s: float,
                 blackboard: Blackboard | None = None):
        """Initialize mission timing, control state, and its permitted phase graph."""
        try:
            self.mode = Mode(mode)
        except (TypeError, ValueError) as error:
            raise ValueError('mode must be auto or manual') from error
        self.blackboard = blackboard if blackboard is not None else Blackboard()
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
        self.airborne_phases = AIRBORNE_PHASES
        self._requested_phase: Phase | None = None
        self.machine = Machine(model=self, states=list(Phase), initial=Phase.PREARM,
                               model_attribute='phase', auto_transitions=False,
                               ignore_invalid_triggers=True, send_event=True,
                               after_state_change=self._on_phase_changed)
        # Safety transitions are first, followed by the normal mission path.
        for source in Phase:
            if source != Phase.FAULT:
                self._add_transition(source, Phase.FAULT)
        for source in (Phase.ARMING, Phase.ARMED):
            self._add_transition(source, Phase.DISARMING)
        for source in (Phase.TAKEOFF, Phase.HOVER):
            self._add_transition(source, Phase.LAND)
        for source, destination in SHARED_PHASE_EDGES:
            self._add_transition(source, destination)

    def _add_transition(self, source: Phase, destination: Phase) -> None:
        """Register a legal phase edge for the machine's resolve trigger."""
        self.machine.add_transition('resolve', source, destination,
                                    conditions=self._wants_phase(destination))

    @staticmethod
    def _wants_phase(destination: Phase):
        """Build a guard that matches the destination requested by the mission."""
        return lambda event: event.model._requested_phase == destination

    def _enter(self, phase: Phase, now: float) -> None:
        """Resolve a phase change now, rejecting edges absent from the graph."""
        if phase == self.phase:
            return
        self._requested_phase = phase
        try:
            if not self.resolve(now=now):
                raise ValueError(f'Invalid mission transition: {self.phase.value} -> {phase.value}')
        finally:
            self._requested_phase = None

    def _on_phase_changed(self, event) -> None:
        """Record phase entry and reset control or dwell state when required."""
        previous = Phase[event.transition.source]
        self.entered_at = event.kwargs['now']
        if self.phase == Phase.LAND or (self.phase == Phase.TAKEOFF and previous != Phase.HOVER):
            self.controller.reset()
            self.last_altitude_at = 0.0
        if self.phase == Phase.TAKEOFF:
            self.near_hover_at = None
        if self.phase == Phase.LAND:
            self.landing_dwell_at = None
        if self.phase == Phase.FAULT:
            color = 'red'
        elif self.phase in (Phase.LAND, Phase.DISARMING):
            color = 'yellow'
        elif self.phase == Phase.DISARMED:
            color = 'green'
        else:
            color = 'cyan'
        logger.opt(colors=True).info(
            f'mission={{}} state {{}} -> <{color}>{{}}</{color}> at={{:.3f}}',
            self.mission_kind, previous.value, self.phase.value, self.entered_at)

    @property
    def readiness(self) -> Readiness:
        """Return the View protocol's readiness code for the current phase."""
        if self.phase == Phase.DISARMED:
            return Readiness.DISARMED_IDLE
        if self.phase == Phase.ARMED:
            return Readiness.ARMED_IDLE
        if self.phase in self.airborne_phases:
            return Readiness.FLYING
        if self.phase == Phase.FAULT:
            return Readiness.RESET_REQUIRED
        return Readiness.UNAVAILABLE


    def fault(self, reason: str, now: float) -> None:
        """Record the failure reason and enter the disarmed-command fault phase."""
        self.fault_reason = reason
        self._enter(Phase.FAULT, now)

    def request(self, action: Action, now: float, altitude_m: float | None = None,
                height_m: float | None = None, *, automatic: bool = False) -> None:
        """Apply an operator or automatic action after mode and phase checks.

        ``action`` must be an ``Action`` member. Accepted actions change phase
        synchronously; Start saves the measured altitude as the launch datum.
        Invalid requests raise ``ValueError``.
        """
        if not isinstance(action, Action):
            raise ValueError('Unsupported flight action')
        if self.phase == Phase.FAULT:
            raise ValueError('App fault; restart required')
        if self.mode == Mode.AUTO and self.started and action in (Action.ARM, Action.START):
            raise ValueError('Automatic mission permits one attempt per process')
        if action == Action.HEIGHT:
            raise ValueError('Height is fixed by the mission profile')
        elif action == Action.ARM:
            if automatic != (self.mode == Mode.AUTO):
                raise ValueError('Arm requires the selected approval mode')
            if self.phase != Phase.DISARMED:
                raise ValueError('App is not disarmed and ready')
            self._enter(Phase.ARMING, now)
        elif action == Action.START:
            if automatic != (self.mode == Mode.AUTO):
                raise ValueError('Start requires the selected approval mode')
            if self.phase != Phase.ARMED or altitude_m is None or not isfinite(altitude_m):
                raise ValueError('Confirmed arm and fresh altitude required')
            self.ground_altitude_m = altitude_m
            self.started = True
            self._enter(Phase.TAKEOFF, now)
        elif action == Action.LAND:
            if self.phase not in self.airborne_phases - {Phase.LAND}:
                raise ValueError('Active flight required to land')
            self._enter(Phase.LAND, now)
        elif action == Action.DISARM:
            if self.phase == Phase.ARMED:
                self._enter(Phase.DISARMING, now)
            elif self.phase != Phase.DISARMED:
                raise ValueError('Emergency confirmation required while flying')
        elif action == Action.EMERGENCY:
            self.fault('Emergency disarm requested', now)
        else:
            raise ValueError('Unsupported flight action')



    def step(self, context: StepContext) -> Command:
        """Advance one control tick using fresh MAVLink altitude and arm feedback.

        Resolve any permitted phase change, then return the RC command for the
        resulting phase. ``context.now_s`` and the MAVLink receive times use
        monotonic seconds; altitude is measured, not height above launch.
        View liveness gates manual flight only. Invalid altitude raises
        ``ValueError`` before the phase or controller is updated.
        """
        if context.blackboard is not self.blackboard:
            raise ValueError('Mission step uses a different blackboard')
        self._validate_altitude(context)
        if self.entered_at is None:
            self.entered_at = context.now_s
        self._handle_view_loss(context)
        self._advance_phase(context)
        return self._flight_command()

    def on_command_sent(self, context: StepContext, command: Command) -> None:
        """Let a mission observe the RC command after App sends it."""
        pass

    @staticmethod
    def _validate_altitude(context: StepContext) -> None:
        """Require a finite altitude received within the last monotonic second."""
        now = context.now_s
        altitude_m = context.altitude_m
        altitude_at = context.altitude_at_s
        if altitude_m is None or not isfinite(altitude_m) or altitude_at <= 0 or altitude_at > now or now - altitude_at > 1.0:
            raise ValueError('Fresh finite MAVLink altitude required')

    def _handle_view_loss(self, context: StepContext) -> None:
        """Disarm idle or land airborne manual flight when View is absent."""
        if self.mode != Mode.MANUAL or context.view_alive:
            return
        if self.phase in (Phase.ARMING, Phase.ARMED):
            self._enter(Phase.DISARMING, context.now_s)
        elif self.phase in self.airborne_phases - {Phase.LAND}:
            self._enter(Phase.LAND, context.now_s)

    def _advance_phase(self, context: StepContext) -> None:
        """Advance the current phase from arm feedback, timers, and altitude."""
        now = context.now_s
        firmware_armed = context.firmware_armed
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
            self._advance_altitude_phase(context)
        elif self.phase == Phase.DISARMING:
            if not firmware_armed:
                self._enter(Phase.DISARMED, now)
                self.controller.reset()
            elif elapsed > DISARM_TIMEOUT_S:
                self.fault('Disarm not confirmed', now)

    def _advance_altitude_phase(self, context: StepContext) -> None:
        """Update airborne altitude control and takeoff, hover, or land state."""
        if not context.firmware_armed:
            self.fault('Vehicle unexpectedly disarmed', context.now_s)
            return
        self._update_altitude_control(context)
        if self.phase == Phase.TAKEOFF:
            self._advance_takeoff(context)
        elif self.phase == Phase.HOVER:
            if not self._near_hover(context):
                self._enter(Phase.TAKEOFF, context.now_s)
            elif context.now_s - self.entered_at >= self.hover_duration_s:
                self._on_hover_complete(context)
        else:
            self._advance_landing(context)
        self._check_phase_timeout(context)

    def _update_altitude_control(self, context: StepContext) -> None:
        """Recompute throttle only for a newer altitude sample."""
        altitude_at = context.altitude_at_s
        if altitude_at <= self.last_altitude_at:
            return
        landing = self.phase == Phase.LAND
        target = self.ground_altitude_m + (0.0 if landing else self.height_m)
        self.controller.update(target, context.altitude_m, altitude_at, landing=landing,
                               low_height_takeoff=(self.phase == Phase.TAKEOFF
                                                   and self.height_m < LOW_HEIGHT_TAKEOFF_THRESHOLD_M))
        self.last_altitude_at = altitude_at

    def _near_hover(self, context: StepContext) -> bool:
        """Check the height band and filtered vertical-speed hover limits."""
        hover_band = min(0.5, max(0.03, self.height_m * 0.25))
        return (self.controller.speed_ready
                and abs(context.altitude_m - (self.ground_altitude_m + self.height_m)) <= hover_band
                and abs(self.controller.speed) <= HOVER_SPEED_M_S)

    def _advance_takeoff(self, context: StepContext) -> None:
        """Enter hover after the vehicle remains near target height for 0.5 s."""
        now = context.now_s
        if not self._near_hover(context):
            self.near_hover_at = None
        elif self.near_hover_at is None:
            self.near_hover_at = now
        elif now - self.near_hover_at >= HOVER_ENTRY_DWELL_S:
            self._enter(Phase.HOVER, now)

    def _advance_landing(self, context: StepContext) -> None:
        """Begin disarming after one second near the launch altitude."""
        now = context.now_s
        landed = (self.controller.speed_ready
                  and abs(context.altitude_m - self.ground_altitude_m) <= LANDING_HEIGHT_M
                  and abs(self.controller.speed) <= LANDING_SPEED_M_S)
        if not landed:
            self.landing_dwell_at = None
        elif self.landing_dwell_at is None:
            self.landing_dwell_at = now
        elif now - self.landing_dwell_at >= LANDING_DWELL_S:
            self._enter(Phase.DISARMING, now)

    def _check_phase_timeout(self, context: StepContext) -> None:
        """Fault a takeoff or landing that exceeds its height-scaled limit."""
        if self.phase == Phase.TAKEOFF:
            limit = max(MIN_PHASE_TIMEOUT_S, PHASE_TIMEOUT_MARGIN_S
                        + self.height_m / CLIMB_TIMEOUT_SPEED_M_S)
        elif self.phase == Phase.LAND:
            limit = max(MIN_PHASE_TIMEOUT_S, PHASE_TIMEOUT_MARGIN_S
                        + self.height_m / LAND_TIMEOUT_SPEED_M_S)
        else:
            return
        if context.now_s - self.entered_at > limit:
            self.fault(f'{self.phase.value} timed out', context.now_s)

    def _flight_command(self) -> Command:
        """Map the resulting phase and controller throttle to an RC command."""
        if self.phase in self.airborne_phases:
            return Command(self.controller.throttle, True)
        return Command(1000, self.phase in (Phase.ARMING, Phase.ARMED))

    def _on_hover_complete(self, context: StepContext) -> None:
        """Let the selected mission choose its phase after timed hover."""
        raise NotImplementedError
