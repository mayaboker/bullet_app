"""Best-effort blackboard sampling and MsgPack/ZMQ publication off the flight loop."""
from dataclasses import fields, is_dataclass
from enum import Enum
from struct import pack
import threading
import time

from loguru import logger
import msgpack
import zmq

from .config import TelemetryPublisherConfig

EPOCH_2000_US = 946_684_800_000_000
MAX_UNIX_NS = 2**64 - 1
SEND_HWM = 16
DROP_WARNING_PERIOD_S = 10.0
SOURCE_TIMED_TOPICS = frozenset((
    'telemetry/attitude', 'telemetry/gps_raw_int',
    'telemetry/global_position_int', 'telemetry/timesync',
))
TELEMETRY_FIELDS = {
    'telemetry/heartbeat': ('heartbeat', 'heartbeat_at', None),
    'telemetry/attitude': ('attitude', 'attitude_at', 'attitude_hz'),
    'telemetry/vfr_hud': ('vfr_hud', 'vfr_hud_at', 'vfr_hud_hz'),
    'telemetry/gps_raw_int': ('gps_raw_int', 'gps_raw_int_at', 'gps_raw_int_hz'),
    'telemetry/global_position_int': ('global_position_int', 'global_position_int_at', 'global_position_int_hz'),
    'telemetry/timesync': ('timesync', None, None),
}
BLACKBOARD_FIELDS = {
    'visual/bbox': ('visual', 'bbox'),
    'simulation/target_contact': ('simulation', 'target_contact'),
    'features/navigation_input': ('features', 'navigation_input'),
    'features/navigation_result': ('features', 'navigation_result'),
    'view/alive': (None, 'view_alive'),
}


def needs_timesync(config: TelemetryPublisherConfig) -> bool:
    """Request vehicle clock alignment only for enabled source-timed topics."""
    return any(topic in SOURCE_TIMED_TOPICS and fps > 0 for topic, fps in config.topics)


def _plain(value):
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {field.name: _plain(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    return value


def _read_topic(blackboard, topic: str):
    """Take one value reference and its related receive/rate metadata."""
    if topic in TELEMETRY_FIELDS:
        field, received_field, rate_field = TELEMETRY_FIELDS[topic]
        snapshot = blackboard.telemetry
        value = getattr(snapshot, field)
        received = getattr(snapshot, received_field) if received_field else (
            value.updated_at if value is not None else None)
        rate = getattr(snapshot, rate_field) if rate_field else None
        return value, received, rate, snapshot.timesync
    domain, field = BLACKBOARD_FIELDS[topic]
    owner = blackboard if domain is None else getattr(blackboard, domain)
    return getattr(owner, field), None, None, blackboard.telemetry.timesync


def _timestamp_ns(topic, value, received_at_s, timesync, mono_ns, wall_ns, bridge_ns=None):
    """Use MAVLink source time when it can be placed on the Unix clock."""
    bridge_ns = wall_ns - mono_ns if bridge_ns is None else bridge_ns
    boot_ns = None
    if topic in ('telemetry/attitude', 'telemetry/global_position_int'):
        raw = value.time_boot_ms
        if type(raw) is int and raw >= 0:
            boot_ns = raw * 1_000_000
    elif topic == 'telemetry/gps_raw_int':
        raw = value.time_usec
        if type(raw) is int and raw > 0:
            if raw >= EPOCH_2000_US:
                epoch_ns = raw * 1_000
                if epoch_ns <= MAX_UNIX_NS:
                    return epoch_ns, 'mavlink'
            else:
                boot_ns = raw * 1_000
    if boot_ns is not None and timesync is not None and timesync.fresh(mono_ns / 1e9):
        epoch_ns = boot_ns + timesync.offset_ns + bridge_ns
        if 0 <= epoch_ns <= MAX_UNIX_NS:
            return epoch_ns, 'mavlink'
    if received_at_s is not None and received_at_s > 0:
        epoch_ns = round(received_at_s * 1e9) + bridge_ns
        if 0 <= epoch_ns <= MAX_UNIX_NS:
            return epoch_ns, 'receive'
    return wall_ns, 'publish'


class TelemetryPublisher:
    """Own a background PUB socket; App never encodes or sends in its tick."""

    def __init__(self, config: TelemetryPublisherConfig, blackboard):
        if not config.enabled:
            raise ValueError('telemetry publisher requires a positive topic FPS')
        self.config = config
        self.blackboard = blackboard
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, name='blackboard-publisher')
        self.start_error = None
        self.dropped = 0
        self._timestamp_cache = {}

    def start(self):
        self.thread.start()
        self.ready.wait()
        if self.start_error is not None:
            self.thread.join()
            raise self.start_error
        return self

    def close(self):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join()

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()

    def _publish(self, socket, topic):
        value, received, rate, timesync = _read_topic(self.blackboard, topic)
        if value is None:
            return
        mono_ns, wall_ns = time.monotonic_ns(), time.time_ns()
        cached = self._timestamp_cache.get(topic) if topic in TELEMETRY_FIELDS else None
        if cached is not None and cached[0] is value and cached[1] == received:
            timestamp_ns, source = cached[2:]
        else:
            timestamp_ns, source = _timestamp_ns(
                topic, value, received, timesync, mono_ns, wall_ns, self._bridge_ns)
            if topic in TELEMETRY_FIELDS:
                self._timestamp_cache[topic] = (value, received, timestamp_ns, source)
        payload = {'schema_version': 1, 'value': _plain(value), 'timestamp_source': source}
        if received is not None:
            payload['received_at_s'] = received
        if rate is not None:
            payload['rate_hz'] = rate
        try:
            socket.send_multipart((topic.encode(), pack('<Q', timestamp_ns),
                                   msgpack.packb(payload, use_bin_type=True)), flags=zmq.NOBLOCK)
        except zmq.Again:
            self.dropped += 1

    def _run(self):
        started = False
        try:
            with zmq.Context() as context:
                with context.socket(zmq.PUB) as socket:
                    socket.setsockopt(zmq.LINGER, 0)
                    socket.setsockopt(zmq.SNDHWM, SEND_HWM)
                    socket.bind(self.config.endpoint)
                    self._bridge_ns = time.time_ns() - time.monotonic_ns()
                    periods = {topic: 1 / fps for topic, fps in self.config.topics if fps > 0}
                    due = {topic: time.monotonic() for topic in periods}
                    warned_at = time.monotonic()
                    started = True
                    self.ready.set()
                    while not self.stop_event.is_set():
                        now = time.monotonic()
                        for topic, deadline in due.items():
                            if now >= deadline:
                                self._publish(socket, topic)
                                due[topic] = now + periods[topic]
                        if self.dropped and now - warned_at >= DROP_WARNING_PERIOD_S:
                            logger.warning('blackboard publisher dropped {} messages', self.dropped)
                            self.dropped = 0
                            warned_at = now
                        self.stop_event.wait(max(0, min(due.values()) - time.monotonic()))
        except Exception as error:
            if started:
                logger.error('blackboard publisher stopped: {}', error)
            else:
                self.start_error = error
        finally:
            self.ready.set()
