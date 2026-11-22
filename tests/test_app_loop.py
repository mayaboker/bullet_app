"""Safety and side-effect ordering at the App runtime boundary."""
from types import SimpleNamespace

from bullet_app import app
from bullet_app.blackboard import StepContext
from bullet_app.missions import Command, Mode, Phase
from bullet_app.runtime_diagnostics import RuntimeDiagnostics


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


def test_auto_run_publishes_before_result_and_final_disarm(monkeypatch):
    events = []

    class Resource:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

    class Telemetry(Resource):
        def __init__(self, *_):
            pass

        def set_message_rate(self, *_):
            pass

        def snapshot(self):
            return SimpleNamespace(
                heartbeat=SimpleNamespace(base_mode=0), heartbeat_at=100.0,
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

    class Diagnostics(RuntimeDiagnostics):
        def observe_events(self, *args, **kwargs):
            events.append('diagnostics')
            super().observe_events(*args, **kwargs)

        def observe_periodic(self, *args, **kwargs):
            events.append('periodic')
            super().observe_periodic(*args, **kwargs)

    monkeypatch.setattr(app, 'MavlinkTelemetry', Telemetry)
    monkeypatch.setattr(app, 'TcpTransport', lambda *_: Resource())
    monkeypatch.setattr(app, 'ViewLink', View)
    monkeypatch.setattr(app, 'create_mission', lambda *_: Mission())
    monkeypatch.setattr(app, 'RuntimeDiagnostics', Diagnostics)
    monkeypatch.setattr(app, 'read_board_identifier', lambda *_: 'SITL')
    monkeypatch.setattr(app.time, 'monotonic', lambda: 100.0)
    monkeypatch.setattr(app, '_send_command', lambda _transport, command: events.append(f'send {command.throttle}'))

    options = SimpleNamespace(mission='hover-land', mavlink_endpoint='', msp_host='', msp_port=0)
    assert app.run_mission(options) == 0
    assert events == ['poll', 'step', 'send 1300', 'publish', 'diagnostics',
                      'send 1000', 'send 1000', 'send 1000']
