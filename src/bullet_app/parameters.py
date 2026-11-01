"""MAVLink-thread parameter service and owner-thread application helpers."""
from collections import deque
from dataclasses import asdict, dataclass
from queue import Queue, Empty, Full
from threading import Lock
from uuid import uuid4

from loguru import logger
from bullet_msgs.parameters import (BY_ID, CATALOG_ID, CATALOG_VERSION, REVISION_ID,
    PARAMETER_IDS, INT64, profile_values, profile_dict, encode_value, decode_value, raw_value)
from los_tracking import (NavigatorConfig, CameraCalibration, VehicleConfig, Tuning,
                          ControlConfig, ControlConnection)
from .mission import Mission
from .config_snapshot import configuration_snapshot

PARAMETER_STREAM_PERIOD_S = .05
PARAMETER_QUEUE_CAPACITY = 64
PARAMETER_REQUESTS_PER_TICK = 4
ACK_ACCEPTED = 0
ACK_UNSUPPORTED = 1
ACK_FAILED = 2


@dataclass(frozen=True)
class ParameterRequest:
    id: str
    value: object
    requester: tuple[int, int]
    error: str = ''


def config_from_values(values):
    d = profile_dict(values)
    connection = ControlConnection(**d['control'].pop('connection'))
    return NavigatorConfig(camera=CameraCalibration(**d['camera']), vehicle=VehicleConfig(**d['vehicle']),
        target_altitude_m=d['target_altitude_m'], tuning=Tuning(**d['tuning']),
        control=ControlConfig(**d['control'], connection=connection))


class ParameterService:
    """Bounded handoff; only the MAVLink thread writes to the MAVLink socket."""
    def __init__(self, config):
        self.lock = Lock()
        self.values = profile_values(asdict(config))
        self.revision = 0
        self.requests = Queue(PARAMETER_QUEUE_CAPACITY)
        self.replies = Queue(PARAMETER_QUEUE_CAPACITY)
        self.stream = deque()
        self.announcements = set()
        self.next_stream_s = 0.
        self.replace_config(config)

    def snapshot(self):
        with self.lock:
            return {CATALOG_ID: CATALOG_VERSION, REVISION_ID: self.revision, **self.values}

    def replace_config(self, config):
        values = profile_values(asdict(config))
        # Ensure the entire advertised profile can be transported before committing it.
        for id, value in values.items():
            encode_value(value, BY_ID[id].wire_type)
        with self.lock:
            changed = values != self.values
            if changed:
                self.announcements.update(id for id, value in values.items() if self.values.get(id) != value)
                self.announcements.add(REVISION_ID)
                self.revision += 1
                self.values = values
            return self.revision, changed

    def handle(self, message, mav):
        kind = message.get_type()
        if kind == 'PARAM_EXT_REQUEST_LIST':
            self.stream = deque(PARAMETER_IDS)
        elif kind == 'PARAM_EXT_REQUEST_READ':
            id = str(message.param_id).rstrip('\0')
            if message.param_index >= 0:
                id = PARAMETER_IDS[message.param_index] if message.param_index < len(PARAMETER_IDS) else ''
            self.send_value(mav, id)
        elif kind == 'PARAM_EXT_SET':
            id = str(message.param_id).rstrip('\0')
            p = BY_ID.get(id)
            error, value = '', None
            try:
                if p is None or p.wire_type != message.param_type:
                    raise ValueError('unknown parameter or incorrect type')
                value = decode_value(raw_value(message), message.param_type)
            except (ValueError, TypeError, OverflowError) as exc:
                error = str(exc)
            request = ParameterRequest(id, value, (message.get_srcSystem(), message.get_srcComponent()), error)
            try:
                self.requests.put_nowait(request)
            except Full:
                self.send_ack(mav, id, ACK_FAILED)
                logger.warning('Parameter request rejected: {} queue full', id)

    def send_value(self, mav, id):
        values = self.snapshot()
        if id in values:
            wire_type = BY_ID[id].wire_type if id in BY_ID else INT64
            mav.param_ext_value_send(id.encode(), encode_value(values[id], wire_type), wire_type,
                                     len(PARAMETER_IDS), PARAMETER_IDS.index(id))

    def send_ack(self, mav, id, result):
        values = self.snapshot()
        p = BY_ID.get(id)
        wire_type = p.wire_type if p else INT64
        value = values.get(id, 0)
        mav.param_ext_ack_send(id.encode(), encode_value(value, wire_type), wire_type, result)

    def poll(self, mav, now_s):
        try:
            id, result, text = self.replies.get_nowait()
        except Empty:
            pass
        else:
            self.send_ack(mav, id, result)
            if result == ACK_ACCEPTED:
                self.send_value(mav, id)
            mav.statustext_send(6 if result == ACK_ACCEPTED else 4, text.encode('ascii', errors='replace')[:50])
        if self.stream and now_s >= self.next_stream_s:
            self.send_value(mav, self.stream.popleft())
            self.next_stream_s = now_s + PARAMETER_STREAM_PERIOD_S
        elif now_s >= self.next_stream_s:
            with self.lock:
                id = self.announcements.pop() if self.announcements else None
            if id is not None:
                self.send_value(mav, id)
                self.next_stream_s = now_s + PARAMETER_STREAM_PERIOD_S

    def take_requests(self):
        # Reserve reply capacity before removing work, so control never blocks on output.
        for _ in range(min(PARAMETER_REQUESTS_PER_TICK, self.replies.maxsize - self.replies.qsize())):
            try:
                yield self.requests.get_nowait()
            except Empty:
                return


def apply_request(runtime, mission, request, platform, msp, now_s, revisions):
    """Called exclusively by the control loop. Log/record before acknowledging."""
    service = runtime.mavlink.parameters
    p = BY_ID.get(request.id)
    old = service.snapshot().get(request.id)
    effective, result, reason = old, ACK_FAILED, ''
    changed = False
    event_phase = mission.phase
    event_source = None if mission.feedback is None else mission.feedback['source']
    try:
        if request.error or p is None:
            raise ValueError(request.error or 'unknown parameter')
        stopped = (mission.phase in ('READY', 'COMPLETED', 'FAULT') and mission.feedback is not None
                   and mission.feedback.get('state') != 'running')
        if not stopped and (mission.phase == 'LOS_COMMIT' or mission.navigator.state.value == 'commit'):
            raise ValueError('cannot edit during COMMIT')
        if not p.live and not stopped:
            raise ValueError('parameter requires stopped mission')
        values = profile_values(asdict(runtime.config))
        values[p.id] = request.value
        candidate = config_from_values(values)
        # Validate transportability before mutating the mission.
        for id, value in profile_values(asdict(candidate)).items():
            encode_value(value, BY_ID[id].wire_type)
        if candidate != runtime.config:
            if p.live and not stopped:
                mission.navigator.apply_live_tuning(candidate.tuning)
                mission.hold.apply_live_tuning(candidate.tuning)
                mission.config = candidate
            else:
                replacement = Mission(candidate, hover_check=runtime.hover_check)
                if platform is not None:
                    replacement.configure(platform)
                new_msp = msp
                if candidate.control != runtime.config.control:
                    new_msp = None
                    if platform is not None and platform.get('backend') == 'betaflight':
                        from .msp_flight import MspFlight
                        new_msp = MspFlight(connection=candidate.control.connection)
                        new_msp.start()
                    if msp is not None:
                        msp.close()
                mission, msp = replacement, new_msp
            runtime.config = candidate
            _, changed = service.replace_config(candidate)
        effective = request.value
        result = ACK_ACCEPTED
    except (ValueError, TypeError, OSError, OverflowError) as exc:
        reason = str(exc)
    revision = service.snapshot()[REVISION_ID]
    path = p.path if p else request.id
    if result != ACK_ACCEPTED or changed:
        event = dict(schema_version=1, event_id=uuid4().hex, result='applied' if result == ACK_ACCEPTED else 'rejected',
            parameter_id=request.id, parameter_path=path, requested_value=request.value, old_value=old,
            effective_value=effective, reason=reason, requester_system=request.requester[0],
            requester_component=request.requester[1], config_revision=revision, phase=event_phase,
            applied_at_s=now_s, source=event_source)
        runtime.blackboard.enqueue_parameters(event, now_s)
        if result == ACK_ACCEPTED:
            logger.info('Parameter applied: {} {} -> {} phase={} requester={}/{} config_revision={} applied_at_s={}',
                path, old, effective, mission.phase, *request.requester, revision, now_s)
            if event_source is not None:
                runtime.blackboard.write_config_snapshot(configuration_snapshot(
                    runtime.config, event_source, revisions, getattr(runtime, 'config_files', {})), now_s)
        else:
            logger.warning('Parameter rejected: {} requested={} reason={} phase={} requester={}/{}',
                path, request.value, reason, mission.phase, *request.requester)
    text = f'{request.id}={effective}: applied' if result == ACK_ACCEPTED else f'{request.id}: {reason}'
    service.replies.put_nowait((request.id, result, text))
    return mission, msp
