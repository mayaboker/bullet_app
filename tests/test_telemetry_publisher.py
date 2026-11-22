"""Blackboard publication clocks, encoding, and background delivery."""
from dataclasses import replace
from struct import unpack
import socket
import time

from beta_client.entities import Attitude, TelemetrySnapshot
from beta_client.timesync import TimeSyncStatus
from los_tracking.types import Diagnostics, NavigationResult, State, TrackStage
import msgpack
import pytest
import zmq

from bullet_app.blackboard import Blackboard
from bullet_app.config import TelemetryPublisherConfig
from bullet_app.telemetry_publisher import (TelemetryPublisher, _plain, _read_topic,
                                            _timestamp_ns, needs_timesync)


def _attitude(boot_ms=4000):
    return Attitude(boot_ms, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0)


def test_source_clock_uses_timesync_and_falls_back_to_receive_time():
    mono_ns = 10_000_000_000
    wall_ns = 1_700_000_000_000_000_000
    sync = TimeSyncStatus(5_000_000_000, 1_000_000, 5, 10.0)
    assert _timestamp_ns('telemetry/attitude', _attitude(), 9.9, sync, mono_ns, wall_ns) == (
        wall_ns - 1_000_000_000, 'mavlink')
    assert _timestamp_ns('telemetry/attitude', _attitude(), 9.9, None, mono_ns, wall_ns) == (
        wall_ns - 100_000_000, 'receive')
    assert _timestamp_ns('visual/bbox', {'capture_ns': 1}, None, sync, mono_ns, wall_ns) == (
        wall_ns, 'publish')


def test_epoch_gps_timestamp_and_enum_conversion():
    from beta_client.entities import GpsRawInt

    gps = GpsRawInt(1_700_000_000_123_456, 3, 0, 0, 0, 0, 0, 0, 0, 10)
    assert _timestamp_ns('telemetry/gps_raw_int', gps, 9.9, None,
                         10_000_000_000, 1_700_000_001_000_000_000) == (
        1_700_000_000_123_456_000, 'mavlink')
    boot_gps = replace(gps, time_usec=4_000_000)
    sync = TimeSyncStatus(5_000_000_000, 1_000_000, 5, 10.0)
    assert _timestamp_ns('telemetry/gps_raw_int', boot_gps, 9.9, sync,
                         10_000_000_000, 1_700_000_000_000_000_000) == (
        1_699_999_999_000_000_000, 'mavlink')
    result = NavigationResult(1, 10.0, 10.02, State.TRACK, TrackStage.BEARING,
                              'tracking', None, Diagnostics())
    encoded = msgpack.packb(_plain(result), use_bin_type=True)
    decoded = msgpack.unpackb(encoded, raw=False)
    assert decoded['state'] == 'track' and decoded['stage'] == 'bearing'


def test_blackboard_topics_include_metadata_and_false_values():
    board = Blackboard()
    board.reset()
    board.telemetry = TelemetrySnapshot(attitude=_attitude(), attitude_at=9.9, attitude_hz=49.0)
    value, received, rate, _ = _read_topic(board, 'telemetry/attitude')
    assert value == board.telemetry.attitude and (received, rate) == (9.9, 49.0)
    assert _read_topic(board, 'simulation/target_contact')[0] is False
    assert _read_topic(board, 'features/navigation_result')[0] is None


def test_background_publisher_repeats_latest_and_keeps_source_header():
    board = Blackboard()
    board.reset()
    board.telemetry = TelemetrySnapshot(attitude=_attitude(), attitude_at=10.0,
                                         timesync=TimeSyncStatus(5_000_000_000, 1_000_000, 5, 10.0))
    with socket.socket() as reserved:
        reserved.bind(('127.0.0.1', 0))
        endpoint = f'tcp://127.0.0.1:{reserved.getsockname()[1]}'
    config = TelemetryPublisherConfig(endpoint, (('telemetry/attitude', 20.0),))
    assert needs_timesync(config)
    with zmq.Context() as context, context.socket(zmq.SUB) as subscriber:
        subscriber.setsockopt(zmq.SUBSCRIBE, b'telemetry/attitude')
        subscriber.setsockopt(zmq.RCVTIMEO, 2000)
        subscriber.connect(endpoint)
        with TelemetryPublisher(config, board):
            frames = [subscriber.recv_multipart() for _ in range(2)]
    assert all(len(parts) == 3 and len(parts[1]) == 8 for parts in frames)
    assert frames[0][0] == frames[1][0] == b'telemetry/attitude'
    assert unpack('<Q', frames[0][1])[0] == unpack('<Q', frames[1][1])[0]
    values = [msgpack.unpackb(parts[2], raw=False) for parts in frames]
    assert values[0]['value']['time_boot_ms'] == values[1]['value']['time_boot_ms'] == 4000
    assert values[0]['received_at_s'] == 10.0


def test_publisher_bind_error_fails_startup():
    board = Blackboard()
    with zmq.Context() as context, context.socket(zmq.PUB) as occupied:
        port = occupied.bind_to_random_port('tcp://127.0.0.1')
        config = TelemetryPublisherConfig(f'tcp://127.0.0.1:{port}', (('view/alive', 1.0),))
        with pytest.raises(zmq.ZMQError):
            TelemetryPublisher(config, board).start()


def test_nonblocking_send_drop_does_not_escape_worker():
    board = Blackboard()
    board.reset()
    config = TelemetryPublisherConfig('tcp://127.0.0.1:5557', (('view/alive', 1.0),))
    publisher = TelemetryPublisher(config, board)
    publisher._bridge_ns = 0

    class FullSocket:
        def send_multipart(self, frames, *, flags):
            assert flags == zmq.NOBLOCK
            raise zmq.Again()

    publisher._publish(FullSocket(), 'view/alive')
    assert publisher.dropped == 1


def test_repeated_telemetry_keeps_source_timestamp_after_sync_expires():
    board = Blackboard()
    board.reset()
    now_ns = time.monotonic_ns()
    attitude = _attitude()
    board.telemetry = TelemetrySnapshot(
        attitude=attitude, attitude_at=now_ns / 1e9,
        timesync=TimeSyncStatus(now_ns - 4_000_000_000, 1_000_000, 5, now_ns / 1e9))
    config = TelemetryPublisherConfig('tcp://127.0.0.1:5557', (('telemetry/attitude', 1.0),))
    publisher = TelemetryPublisher(config, board)
    publisher._bridge_ns = time.time_ns() - now_ns

    class Socket:
        frames = []

        def send_multipart(self, parts, *, flags):
            self.frames.append(parts)

    socket = Socket()
    publisher._publish(socket, 'telemetry/attitude')
    board.telemetry = replace(board.telemetry, timesync=None)
    publisher._publish(socket, 'telemetry/attitude')
    assert socket.frames[0][1] == socket.frames[1][1]
    assert msgpack.unpackb(socket.frames[1][2], raw=False)['timestamp_source'] == 'mavlink'
