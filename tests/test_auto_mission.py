from types import SimpleNamespace

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import common

from bullet_app.cli import AppOptions, CliParser
from bullet_app.flight import Command, FlightMission, Phase
from bullet_app.mission import _send_command
from bullet_app.view_link import ViewLink, _command
from bullet_app.target import TargetMission


def ready(mode='auto', ground=42.0):
    mission = FlightMission(mode)
    assert mission.step(100, ground, 100, False, True) == Command(1000, False)
    mission.step(112, ground, 112, False, True)
    assert mission.phase == Phase.DISARMED
    mission.request('arm', 112.1)
    assert mission.step(112.2, ground, 112.2, False, True).arm
    mission.step(112.3, ground, 112.3, True, True)
    assert mission.phase == Phase.ARMED
    mission.request('start', 112.4, ground)
    return mission


def test_auto_waits_for_view_then_completes_and_can_rearm():
    mission = FlightMission('auto')
    with pytest.raises(ValueError):
        mission.request('arm', 1)
    mission.step(100, 42, 100, False, True)
    mission.step(112, 42, 112, False, True)
    assert mission.phase == Phase.DISARMED
    mission = ready()
    mission.step(113, 51.9, 113, True, True)
    mission.step(113.1, 52, 113.1, True, True)
    for now in (113.3, 113.5, 113.7):
        mission.step(now, 52, now, True, True)
    assert mission.phase == Phase.HOVER
    for index in range(1, 60):
        now = 113.7 + index * .2
        mission.step(now, 52, now, True, True)
        if mission.phase == Phase.LAND:
            break
    assert mission.phase == Phase.LAND
    land_start = now + .1
    mission.step(land_start, 42.1, land_start, True, True)
    for index in range(1, 18):
        now = land_start + index * .1
        mission.step(now, 42.1, now, True, True)
    assert mission.phase == Phase.DISARMING
    mission.step(now + .1, 42.1, now + .1, False, True)
    assert mission.phase == Phase.DISARMED
    mission.request('arm', 126)
    assert mission.phase == Phase.ARMING


def test_manual_height_hold_land_and_link_loss():
    mission = FlightMission('manual')
    mission.step(100, 42, 100, False, True)
    mission.step(112, 42, 112, False, True)
    mission.request('height', 112.1, height_m=100)
    assert mission.height_m == 100
    with pytest.raises(ValueError):
        mission.request('height', 112.1, height_m=100.1)
    mission.request('arm', 112.2)
    mission.step(112.3, 42, 112.3, True, True)
    mission.request('start', 112.4, 42)
    mission.step(113, 142, 113, True, True)
    mission.step(113.1, 142, 113.1, True, True)
    for now in (113.3, 113.5, 113.7):
        mission.step(now, 142, now, True, True)
    assert mission.phase == Phase.HOVER
    mission.step(200, 142, 200, True, True)
    assert mission.phase == Phase.TAKEOFF  # long sample gap invalidates hover speed
    mission.request('land', 200.1)
    assert mission.phase == Phase.LAND
    mission.step(200.2, 141, 200.2, True, False)
    assert mission.phase == Phase.LAND


def test_view_loss_lands_or_disarms_and_fault_blocks_restart():
    mission = ready()
    mission.step(113, 43, 113, True, False)
    assert mission.phase == Phase.LAND
    other = FlightMission('manual')
    other.step(100, 0, 100, False, True)
    other.step(112, 0, 112, False, True)
    other.request('arm', 112.1)
    other.step(112.2, 0, 112.2, True, True)
    other.step(113, 0, 113, True, False)
    assert other.phase == Phase.DISARMING
    other.fault('telemetry stale', 114)
    with pytest.raises(ValueError, match='fault'):
        other.request('arm', 115)


def test_command_mapping_and_cli():
    sent = []
    transport = SimpleNamespace(send=sent.append, drain_responses=lambda: None)
    _send_command(transport, Command(1400, True))
    frame = sent[0]
    channels = tuple(int.from_bytes(frame[5 + 2*i:7 + 2*i], 'little') for i in range(8))
    assert channels == (1500, 1500, 1400, 1500, 2000, 2000, 1000, 1000)
    assert CliParser().parse([]) == AppOptions()
    assert CliParser().parse(['--mode', 'manual']).mode == 'manual'
    message = SimpleNamespace(command=21, **{f'param{i}': 0.0 for i in range(1, 8)})
    assert _command(message) == ('land', None)


def test_view_link_rejects_wrong_source_and_acks_commands():
    class Message:
        command = 400
        target_system = target_component = 1
        param1 = 1.0
        param2 = param3 = param4 = param5 = param6 = param7 = 0.0

        def __init__(self, kind, source=(255, 190)):
            self.kind, self.source = kind, source

        def get_type(self):
            return self.kind

        def get_srcSystem(self):
            return self.source[0]

        def get_srcComponent(self):
            return self.source[1]

    mission = FlightMission('auto')
    mission.step(100, 0, 100, False, True)
    mission.step(112, 0, 112, False, True)
    messages = iter((Message('COMMAND_LONG', (2, 2)), Message('HEARTBEAT'),
                     Message('COMMAND_LONG'), None))
    acks = []
    link = ViewLink()
    link.connection = SimpleNamespace(recv_match=lambda **kwargs: next(messages),
                                      mav=SimpleNamespace(command_ack_send=lambda *args: acks.append(args)))
    link.poll(mission, 112.1, 0)
    assert mission.phase == Phase.ARMING
    assert acks == [(400, 0, 0, 1, 255, 190)]


def test_view_link_uses_one_udp_socket_for_stream_command_and_ack():
    view = mavutil.mavlink_connection('udpin:127.0.0.1:0')
    view.mav = common.MAVLink(view, srcSystem=255, srcComponent=190)
    view.WIRE_PROTOCOL_VERSION = '2.0'
    view.first_byte = False
    try:
        port = view.port.getsockname()[1]
        mission = FlightMission('auto')
        mission.step(100, 0, 100, False, True)
        mission.step(112, 0, 112, False, True)
        sample = SimpleNamespace(heartbeat=None, heartbeat_at=0, attitude=None, vfr_hud=None)
        with ViewLink(endpoint=f'udpout:127.0.0.1:{port}') as link:
            link.publish(mission, sample, 112.1, False)
            heartbeat = view.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            assert heartbeat is not None and heartbeat.get_srcSystem() == 1
            view.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                    mavutil.mavlink.MAV_STATE_ACTIVE, 3)
            view.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            link.poll(mission, 112.2, 0)
            ack = view.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            assert ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            assert ack.command == 400 and ack.result_param2 == 1
            assert mission.phase == Phase.ARMING
    finally:
        view.close()


def test_mavlink_status_names_fit_wire_limit_and_denied_start_reports_reason():
    mission = FlightMission('auto')
    sent = []
    link = ViewLink()
    link.connection = SimpleNamespace(mav=SimpleNamespace(
        named_value_float_send=lambda _, name, value: sent.append((name, value))))
    link.last_heartbeat_at = link.last_telemetry_at = 1.
    link.last_phase = mission.phase
    sample = SimpleNamespace(heartbeat=None, heartbeat_at=0, attitude=None, vfr_hud=None)
    link.publish(mission, sample, 1., False)
    assert all(len(name) <= 10 for name, _ in sent)
    assert (b'APP_MSN', 1.) in sent and (b'VIS_READY', 1.) in sent

    target = TargetMission(CliParser().parse(['--mission', 'target-hit']).mission_config)
    target.phase = Phase.ARMED
    command = SimpleNamespace(get_type=lambda: 'COMMAND_LONG', get_srcSystem=lambda: 255,
        get_srcComponent=lambda: 190, target_system=1, target_component=1, command=300,
        **{f'param{i}': 0. for i in range(1, 8)})
    messages = iter((command, None))
    acks, statuses = [], []
    link.connection = SimpleNamespace(recv_match=lambda **_: next(messages), mav=SimpleNamespace(
        command_ack_send=lambda *args: acks.append(args),
        statustext_send=lambda *args: statuses.append(args)))
    link.last_view_at = 1.
    link.poll(target, 1.1, 10.)
    assert acks[0][1] == mavutil.mavlink.MAV_RESULT_DENIED
    assert b'Live target detector required' in statuses[0][1]
