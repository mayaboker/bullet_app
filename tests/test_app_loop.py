"""Safety and side-effect ordering at the App runtime boundary."""
from types import SimpleNamespace

import pytest

from bullet_app import app
from bullet_app.blackboard import Blackboard, StepContext
from bullet_app.config import DEFAULT_CONFIG
from bullet_app.missions import Command, Mode, Phase
from bullet_app.runtime_diagnostics import RuntimeDiagnostics


def test_simulation_contact_is_polled_independently_of_tracker():
    events = []
    contact = {'target_id': 'red_truck', 'drone_id': 'drone_1'}
    mission = SimpleNamespace(observe_simulation_contact=lambda value: events.append(value))
    simulation = SimpleNamespace(poll=lambda: contact)
    view = SimpleNamespace(poll=lambda *_: None, view_alive=lambda _: True)
    board = Blackboard()
    app._collect_inputs(mission, StepContext(board, 1.0), None, simulation, view)
    assert events == [contact]
    assert board.view_alive


def test_stale_telemetry_faults_and_uses_safe_command():
    from bullet_app.flight import FlightMission

    mission = FlightMission(Mode.AUTO)
    board = mission.blackboard
    board.telemetry = SimpleNamespace(
        heartbeat=SimpleNamespace(base_mode=0), heartbeat_at=5.0,
        vfr_hud=SimpleNamespace(alt=2.0), vfr_hud_at=10.0,
    )

    command = app._step_with_safety(mission, StepContext(board, 10.0), connected_at=0.0)

    assert command == Command(1000, False)
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'SITL telemetry stale'


def test_missing_altitude_keeps_startup_allowance_then_faults():
    from bullet_app.flight import FlightMission

    mission = FlightMission(Mode.AUTO)
    board = mission.blackboard
    board.telemetry = SimpleNamespace(
        heartbeat=SimpleNamespace(base_mode=0), heartbeat_at=5.0,
        vfr_hud=None, vfr_hud_at=0.0,
    )
    assert app._step_with_safety(mission, StepContext(board, 5.0), 0.0) == Command(1000, False)
    assert mission.phase == Phase.PREARM

    board.telemetry.heartbeat_at = 11.0
    assert app._step_with_safety(mission, StepContext(board, 11.0), 0.0) == Command(1000, False)
    assert mission.phase == Phase.FAULT


@pytest.mark.parametrize('serial', (False, True))
@pytest.mark.parametrize('stale', (False, True))
def test_auto_run_publishes_before_result_and_final_disarm(monkeypatch, tmp_path, serial, stale):
    events = []
    connections = []

    class Resource:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

    class Telemetry(Resource):
        def __init__(self, endpoint, *, baud=None):
            connections.append(('mavlink', endpoint, baud))

        def set_message_rate(self, *_):
            pass

        def snapshot(self):
            return SimpleNamespace(
                heartbeat=SimpleNamespace(base_mode=0), heartbeat_at=50.0 if stale else 100.0,
                vfr_hud=SimpleNamespace(alt=2.0), vfr_hud_at=100.0,
                attitude_hz=50.0, vfr_hud_hz=50.0,
            )

    class View(Resource):
        def poll(self, *_):
            events.append('poll')

        def view_alive(self, *_):
            return True

        def publish(self, *_):
            events.append('publish')

        def publish_rate_alert(self, *_):
            events.append('rate alert')

    class Mission:
        mission_kind = 'hover-land'
        mode = Mode.AUTO
        phase = Phase.DISARMED
        started = True
        result = None

        def step(self, context):
            events.append('step')
            return Command(1300, False)

        def fault(self, reason, now):
            self.phase = Phase.FAULT

        def on_command_sent(self, context, command):
            events.append(f'observed {command.throttle}')

    class Diagnostics(RuntimeDiagnostics):
        def observe_events(self, *args, **kwargs):
            events.append('diagnostics')
            super().observe_events(*args, **kwargs)

        def observe_periodic(self, *args, **kwargs):
            events.append('periodic')
            super().observe_periodic(*args, **kwargs)

    monkeypatch.setattr(app, 'MavlinkTelemetry', Telemetry)
    def tcp_transport(host, port):
        connections.append(('msp tcp', host, port))
        return Resource()

    def serial_transport(device, baudrate):
        connections.append(('msp serial', device, baudrate))
        return Resource()

    monkeypatch.setattr(app, 'TcpTransport', tcp_transport)
    monkeypatch.setattr(app, 'SerialTransport', serial_transport)
    monkeypatch.setattr(app, 'ViewLink', View)
    monkeypatch.setattr(app, 'create_mission', lambda *_: Mission())
    monkeypatch.setattr(app, 'RuntimeDiagnostics', Diagnostics)
    monkeypatch.setattr(app, 'read_board_identifier', lambda *_: 'SITL')
    monkeypatch.setattr(app.time, 'monotonic', lambda: 100.0)
    monkeypatch.setattr(app, '_send_command', lambda _transport, command: events.append(f'send {command.throttle}'))

    config_path = DEFAULT_CONFIG
    if serial:
        config_path = tmp_path / 'app.yaml'
        config_path.write_text('betaflight:\n  msp:\n    type: serial\n'
                               '    device: /dev/ttyUSB0\n    baudrate: 115200\n'
                               '  mavlink:\n    type: serial\n'
                               '    endpoint: /dev/ttyUSB1\n    baudrate: 57600\n')
    options = SimpleNamespace(mission='hover-land', config=config_path,
                              mavlink_endpoint=None, msp_host=None, msp_port=None,
                              contact_endpoint=None)
    assert app.run_mission(options) == (1 if stale else 0)
    assert connections == ([('mavlink', '/dev/ttyUSB1', 57600),
                            ('msp serial', '/dev/ttyUSB0', 115200)] if serial else [
                            ('mavlink', 'tcp:127.0.0.1:5762', None),
                            ('msp tcp', '127.0.0.1', 5761)])
    command_events = ['send 1000', 'observed 1000'] if stale else ['step', 'send 1300', 'observed 1300']
    assert events == ['poll', *command_events, 'publish', 'diagnostics',
                      'send 1000', 'send 1000', 'send 1000']
