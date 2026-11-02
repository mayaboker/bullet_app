"""Thread-safe mailbox; mission transitions execute only in the control owner."""
from dataclasses import dataclass
from queue import Queue, Empty, Full
from threading import Lock
from loguru import logger
from bullet_msgs.flight_control import (FlightState, COMMAND_TIMEOUT_S, UNAVAILABLE, ACTION_IDS,
    DISARMED_IDLE, ARMED_IDLE, FLYING, RESET_REQUIRED)

REQUEST_CAPACITY = 32
ACCEPTED, DENIED, FAILED, CANCELLED = 0, 2, 4, 6


@dataclass
class FlightSession:
    """Application-run lifetime; deliberately survives Mission replacement/reset."""
    arming_mode: str = 'automatic'
    operator_control: bool = False
    arm_requested: bool = False
    mission_started: bool = False
    height_override_m: float | None = None
    emergency_identity: tuple | None = None

    def __post_init__(self):
        if self.arming_mode not in ('automatic', 'manual'):
            raise ValueError('arming_mode must be automatic or manual')


class FlightControl:
    def __init__(self):
        self.requests = Queue(REQUEST_CAPACITY)
        self.replies = Queue()
        self.lock = Lock()
        self.state = FlightState()
        self.pending = None

    def submit(self, command, action, value, requester, now):
        request = (command, action, value, requester, now)
        try:
            self.requests.put_nowait(request)
        except Full:
            self.reply(request, FAILED, 'Flight request queue full')

    def reply(self, request, result, text):
        self.replies.put((request[0], request[3], result, text, ACTION_IDS[request[1]]))
        logger.info('Flight command {} result={} {}', request[1], result, text)

    def snapshot(self):
        with self.lock:
            return self.state

    def update(self, mission, now):
        """Drain commands before tick; no socket IO or waits in this method."""
        requests = []
        while True:
            try:
                requests.append(self.requests.get_nowait())
            except Empty:
                break
        # Disarm preempts queued and pending Arm/Start/Apply, even in one batch.
        disarms = [r for r in requests if r[1] in ('disarm', 'emergency')]
        if disarms:
            emergencies = [r for r in disarms if r[1] == 'emergency']
            chosen = (emergencies or disarms)[-1]
            if self.pending is not None:
                self.reply(self.pending, CANCELLED, 'Cancelled by disarm')
                self.pending = None
            for request in requests:
                if request is not chosen:
                    self.reply(request, CANCELLED, 'Cancelled by disarm')
            requests = [chosen]
        for request in requests:
            command, action, value, requester, issued = request
            if now - issued > COMMAND_TIMEOUT_S:
                self.reply(request, FAILED, 'Flight request expired')
                continue
            if self.pending is not None:
                self.reply(request, DENIED, 'Flight command pending')
                continue
            try:
                mission.manual_command(action, value, now)
            except ValueError as error:
                self.reply(request, DENIED, str(error))
                continue
            if action in ('height', 'start'):
                self.reply(request, ACCEPTED, 'Height applied' if action == 'height' else 'Mission started')
            else:
                self.pending = request

    def publish(self, mission, now):
        valid = mission.flight_feedback_valid(now)
        armed = mission.feedback['armed'] if valid else None
        session = mission.flight_session
        locked = session.emergency_identity is not None
        readiness = (UNAVAILABLE if not valid else RESET_REQUIRED if locked else
                     FLYING if session.mission_started or mission.flight_active else
                     ARMED_IDLE if armed and mission.phase == 'ARMED_IDLE' and mission.prearm_ready(now) else
                     UNAVAILABLE if armed else DISARMED_IDLE)
        if readiness == DISARMED_IDLE and not mission.prearm_ready(now):
            readiness = UNAVAILABLE
        state = FlightState(armed, valid, readiness,
            mission.takeoff_height_m if valid else None, now, (1, 1))
        with self.lock:
            self.state = state
        if self.pending is not None:
            action = self.pending[1]
            wants_armed = action == 'arm'
            # Only observations received after issuing the request may confirm it.
            if (valid and armed == wants_armed and mission.arming_observed_at_s > self.pending[4]
                    and (not wants_armed or mission.phase == 'ARMED_IDLE')):
                self.reply(self.pending, ACCEPTED, 'Observed armed idle' if wants_armed else 'Observed disarmed')
                self.pending = None
            elif now - self.pending[4] >= COMMAND_TIMEOUT_S:
                if wants_armed:
                    session.arm_requested = False
                self.reply(self.pending, FAILED, 'Arming feedback timeout; state unknown')
                self.pending = None
