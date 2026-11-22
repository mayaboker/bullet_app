from types import SimpleNamespace

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import common

from bullet_app.cli import AppOptions, CliParser, DEFAULT_TARGET_PROFILE
from bullet_app.flight import Action, Command, FlightMission, Mode, Phase
from bullet_app.app import _advance_automatic_start, _automatic_result, _send_command
from bullet_app.missions import create_mission
from bullet_app.missions.hover_land import HoverLandSettings
from bullet_app.view_link import SITL_CONFIRMED_CUSTOM_MODE, ViewLink, _command
from bullet_app.target import TargetMission
from conftest import context, tick


def ready(mode='manual', ground=42.0):
    mission = FlightMission(mode)
    assert tick(mission, 100, ground, 100, False, True) == Command(1000, False)
    tick(mission, 112, ground, 112, False, True)
    assert mission.phase == Phase.DISARMED
    mission.request(Action.ARM, 112.1, automatic=mode == 'auto')
    assert tick(mission, 112.2, ground, 112.2, False, True).arm
    tick(mission, 112.3, ground, 112.3, True, True)
    assert mission.phase == Phase.ARMED
    mission.request(Action.START, 112.4, ground, automatic=mode == 'auto')
    return mission


def test_auto_runs_without_view_and_stops_after_one_flight():
    mission = FlightMission('auto')
    with pytest.raises(ValueError):
        mission.request(Action.ARM, 1)
    tick(mission, 100, 42, 100, False, True)
    tick(mission, 112, 42, 112, False, True)
    assert mission.phase == Phase.DISARMED
    mission = ready('auto')
    tick(mission, 113, 61.9, 113, True, False)
    tick(mission, 113.1, 62, 113.1, True, False)
    for now in (113.3, 113.5, 113.7):
        tick(mission, now, 62, now, True, False)
    assert mission.phase == Phase.HOVER
    for index in range(1, 60):
        now = 113.7 + index * .2
        tick(mission, now, 62, now, True, False)
        if mission.phase == Phase.LAND:
            break
    assert mission.phase == Phase.LAND
    land_start = now + .1
    tick(mission, land_start, 42.1, land_start, True, True)
    for index in range(1, 18):
        now = land_start + index * .1
        tick(mission, now, 42.1, now, True, True)
    assert mission.phase == Phase.DISARMING
    tick(mission, now + .1, 42.1, now + .1, False, True)
    assert mission.phase == Phase.DISARMED
    with pytest.raises(ValueError, match='one attempt'):
        mission.request(Action.ARM, 126, automatic=True)


def test_manual_approval_uses_profile_height_and_accepts_land():
    mission = FlightMission('manual')
    tick(mission, 100, 42, 100, False, True)
    tick(mission, 112, 42, 112, False, True)
    with pytest.raises(ValueError, match='mission profile'):
        mission.request(Action.HEIGHT, 112.1, height_m=100)
    assert mission.height_m == 20
    mission.request(Action.ARM, 112.2)
    tick(mission, 112.3, 42, 112.3, True, True)
    mission.request(Action.START, 112.4, 42)
    tick(mission, 113, 62, 113, True, True)
    tick(mission, 113.1, 62, 113.1, True, True)
    for now in (113.3, 113.5, 113.7):
        tick(mission, now, 62, now, True, True)
    assert mission.phase == Phase.HOVER
    tick(mission, 120, 62, 120, True, True)
    assert mission.phase == Phase.TAKEOFF  # long sample gap invalidates hover speed
    mission.request(Action.LAND, 120.1)
    assert mission.phase == Phase.LAND
    tick(mission, 120.2, 61, 120.2, True, False)
    assert mission.phase == Phase.LAND


def test_view_loss_lands_or_disarms_and_fault_blocks_restart():
    mission = ready()
    tick(mission, 113, 43, 113, True, False)
    assert mission.phase == Phase.LAND
    other = FlightMission('manual')
    tick(other, 100, 0, 100, False, True)
    tick(other, 112, 0, 112, False, True)
    other.request(Action.ARM, 112.1)
    tick(other, 112.2, 0, 112.2, True, True)
    tick(other, 113, 0, 113, True, False)
    assert other.phase == Phase.DISARMING
    other.fault('telemetry stale', 114)
    with pytest.raises(ValueError, match='fault'):
        other.request(Action.ARM, 115)


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
    assert _command(message) == (Action.LAND, None)
    message.command = 31012
    message.param1 = 12.5
    assert _command(message) == (Action.HEIGHT, 12.5)
    message.param2 = 3.5
    with pytest.raises(ValueError, match='Unsupported'):
        _command(message)


def test_request_requires_an_action_member():
    mission = FlightMission('manual')
    with pytest.raises(ValueError, match='Unsupported flight action'):
        mission.request('arm', 1.)
    target = TargetMission(DEFAULT_TARGET_PROFILE)
    with pytest.raises(ValueError, match='Unsupported flight action'):
        target.request('arm', 1.)


def test_mission_mode_normalizes_cli_text_and_accepts_enum():
    assert FlightMission('auto').mode is Mode.AUTO
    assert FlightMission(Mode.MANUAL).mode is Mode.MANUAL
    assert FlightMission().mode is Mode.MANUAL
    with pytest.raises(ValueError, match='mode must be auto or manual'):
        FlightMission('other')
    with pytest.raises(ValueError, match='mode must be auto or manual'):
        FlightMission(None)


def test_target_height_locks_on_start_and_hover_duration_is_fixed():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.DISARMED
    mission.request_height(12.5)
    assert (mission.height_m, mission.hover_duration_s) == (12.5, 5.)
    with pytest.raises(ValueError, match='height'):
        mission.request_height(0.)
    assert mission.height_m == 12.5
    mission.phase = Phase.ARMED
    mission.request_height(15.)
    assert (mission.height_m, mission.hover_duration_s) == (15., 5.)
    mission.phase = Phase.TAKEOFF
    with pytest.raises(ValueError, match='idle'):
        mission.request_height(10.)
    with pytest.raises(ValueError, match='profile'):
        FlightMission('manual').request_height(10.)
    automatic = TargetMission(DEFAULT_TARGET_PROFILE, mode='auto')
    automatic.phase = Phase.DISARMED
    with pytest.raises(ValueError, match='manual'):
        automatic.request_height(10.)


def test_view_link_applies_target_height_and_acks_denial():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.DISARMED
    acks = []
    messages = []
    for height, hover in ((12.5, 0.), (14., 3.5)):
        messages.append(SimpleNamespace(
            command=31012, target_system=1, target_component=1,
            **{f'param{i}': value for i, value in enumerate((height, hover, 0., 0., 0., 0., 0.), 1)},
            get_type=lambda: 'COMMAND_LONG', get_srcSystem=lambda: 255,
            get_srcComponent=lambda: 190))
    messages.append(None)
    iterator = iter(messages)
    link = ViewLink()
    link.last_view_at = 1.
    link.connection = SimpleNamespace(recv_match=lambda **kwargs: next(iterator),
        mav=SimpleNamespace(command_ack_send=lambda *args: acks.append(args),
                            statustext_send=lambda *args: None))
    link.poll(mission, context(mission, 1.1, 0., 1.1))
    assert (mission.height_m, mission.hover_duration_s) == (12.5, 5.)
    assert [ack[:2] for ack in acks] == [(31012, mavutil.mavlink.MAV_RESULT_ACCEPTED),
                                         (31012, mavutil.mavlink.MAV_RESULT_DENIED)]
    assert [ack[3] for ack in acks] == [5, 0]


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

    mission = FlightMission('manual')
    tick(mission, 100, 0, 100, False, True)
    tick(mission, 112, 0, 112, False, True)
    messages = iter((Message('COMMAND_LONG', (2, 2)), Message('HEARTBEAT'),
                     Message('COMMAND_LONG'), None))
    acks = []
    link = ViewLink()
    link.connection = SimpleNamespace(recv_match=lambda **kwargs: next(messages),
                                      mav=SimpleNamespace(command_ack_send=lambda *args: acks.append(args)))
    link.poll(mission, context(mission, 112.1, 0., 112.1))
    assert mission.phase == Phase.ARMING
    assert acks == [(400, 0, 0, 1, 255, 190)]


def test_view_link_uses_one_udp_socket_for_stream_command_and_ack():
    view = mavutil.mavlink_connection('udpin:127.0.0.1:0')
    view.mav = common.MAVLink(view, srcSystem=255, srcComponent=190)
    view.WIRE_PROTOCOL_VERSION = '2.0'
    view.first_byte = False
    try:
        port = view.port.getsockname()[1]
        mission = FlightMission('manual')
        tick(mission, 100, 0, 100, False, True)
        tick(mission, 112, 0, 112, False, True)
        sample = SimpleNamespace(heartbeat=None, heartbeat_at=0, attitude=None, vfr_hud=None)
        with ViewLink(endpoint=f'udpout:127.0.0.1:{port}') as link:
            mission.blackboard.telemetry = sample
            link.publish(mission, context(mission, 112.1))
            heartbeat = view.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            assert heartbeat is not None and heartbeat.get_srcSystem() == 1
            assert heartbeat.custom_mode == 0
            link.sitl_confirmed = True
            sample.heartbeat = SimpleNamespace(base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            sample.heartbeat_at = 113.2
            link.publish(mission, context(mission, 113.2))
            confirmed = view.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            assert confirmed.custom_mode == SITL_CONFIRMED_CUSTOM_MODE
            assert confirmed.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            assert confirmed.system_status == mavutil.mavlink.MAV_STATE_STANDBY
            view.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                    mavutil.mavlink.MAV_STATE_ACTIVE, 3)
            view.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            link.poll(mission, context(mission, 112.2, 0., 112.2))
            ack = view.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            assert ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            assert ack.command == 400 and ack.result_param2 == 1
            assert mission.phase == Phase.ARMING
    finally:
        view.close()


def test_mavlink_status_names_fit_wire_limit_and_denied_start_reports_reason():
    mission = FlightMission('manual')
    sent = []
    link = ViewLink()
    link.connection = SimpleNamespace(mav=SimpleNamespace(
        named_value_float_send=lambda _, name, value: sent.append((name, value))))
    link.last_heartbeat_at = link.last_telemetry_at = 1.
    link.last_phase = mission.phase
    sample = SimpleNamespace(heartbeat=None, heartbeat_at=0, attitude=None, vfr_hud=None,
                             attitude_hz=49., vfr_hud_hz=48.)
    mission.blackboard.telemetry = sample
    link.publish(mission, context(mission, 1.))
    assert all(len(name) <= 10 for name, _ in sent)
    assert (b'APP_MSN', 1.) in sent and (b'VIS_READY', 1.) in sent
    assert (b'HOVER_S', mission.hover_duration_s) in sent
    assert (b'ATT_HZ', 49.) in sent and (b'ALT_HZ', 48.) in sent

    alerts = []
    link.connection.mav.statustext_send = lambda *args: alerts.append(args)
    link.publish_rate_alert('ATTITUDE', 'low', 32.)
    link.publish_rate_alert('ATTITUDE', 'recovered', 48.)
    assert alerts[0][0] == 4 and b'32.0 Hz' in alerts[0][1]
    assert alerts[1][0] == 6 and b'recovered' in alerts[1][1]

    target = TargetMission(DEFAULT_TARGET_PROFILE)
    target.phase = Phase.DISARMED
    command = SimpleNamespace(get_type=lambda: 'COMMAND_LONG', get_srcSystem=lambda: 255,
        get_srcComponent=lambda: 190, target_system=1, target_component=1, command=400,
        **{f'param{i}': (1. if i == 1 else 0.) for i in range(1, 8)})
    messages = iter((command, None))
    acks, statuses = [], []
    link.connection = SimpleNamespace(recv_match=lambda **_: next(messages), mav=SimpleNamespace(
        command_ack_send=lambda *args: acks.append(args),
        statustext_send=lambda *args: statuses.append(args)))
    link.last_view_at = 1.
    link.poll(target, context(target, 1.1, 10., 1.1))
    assert acks[0][1] == mavutil.mavlink.MAV_RESULT_DENIED
    assert target.phase == Phase.DISARMED
    assert b'TIMESYNC' in statuses[0][1]


def test_hover_profile_and_startup_selection(tmp_path):
    profile = tmp_path / 'hover.yaml'
    profile.write_text('altitude_m: 25\nhover_duration_s: 3\n')
    options = CliParser().parse(['--mission', 'hover-land', '--mission-config', str(profile)])
    mission = create_mission(options)
    assert mission.height_m == 25 and mission.hover_duration_s == 3
    assert mission.mode is Mode.MANUAL
    assert create_mission(CliParser().parse(['--mission', 'target-hit'])).mission_kind == 'target-hit'
    profile.write_text('altitude_m: 101\nhover_duration_s: 3\n')
    with pytest.raises(ValueError, match='altitude'):
        HoverLandSettings.from_yaml(profile)
    profile.write_text('altitude_m: 20\nhover_duration_s: 0\n')
    with pytest.raises(ValueError, match='duration'):
        HoverLandSettings.from_yaml(profile)
    profile.write_text('altitude_m: 20\n')
    with pytest.raises(ValueError, match='requires'):
        HoverLandSettings.from_yaml(profile)


def test_auto_start_waits_for_ready_state_and_exits_after_disarm():
    mission = FlightMission('auto')
    _advance_automatic_start(mission, context(mission, 1., 0., 1.))
    assert mission.phase == Phase.PREARM
    tick(mission, 1., 0., 1., False, False)
    tick(mission, 13., 0., 13., False, False)
    _advance_automatic_start(mission, context(mission, 13., 0., 13.))
    assert mission.phase == Phase.ARMING
    tick(mission, 13.1, 0., 13.1, True, False)
    _advance_automatic_start(mission, context(mission, 13.1, 0., 13.1, armed=True))
    assert mission.phase == Phase.TAKEOFF and mission.started
    assert _automatic_result(mission, True) is None
    mission.phase = Phase.DISARMED
    assert _automatic_result(mission, False) == 0
    mission.result = 'target not found'
    assert _automatic_result(mission, False) == 1
