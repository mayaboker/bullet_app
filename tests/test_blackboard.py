"""Shared observation identity, reset, and step-context contract."""
from beta_client.entities import Heartbeat, TelemetrySnapshot, VfrHud
from pymavlink import mavutil

from bullet_app.blackboard import Blackboard, StepContext
from bullet_app.flight import FlightMission, Phase


def test_process_singleton_reset_clears_observations():
    board = Blackboard()
    assert Blackboard() is board
    board.visual.bbox = {'capture_ns': 123, 'bbox': None}
    board.visual.contact = True
    board.features.navigation_input = object()
    board.features.navigation_result = object()
    board.view_alive = True

    board.reset()

    assert board.telemetry == TelemetrySnapshot()
    assert board.visual.bbox is None and not board.visual.contact
    assert board.features.navigation_input is None
    assert board.features.navigation_result is None
    assert not board.view_alive


def test_context_uses_original_mavlink_values_and_receive_times():
    board = Blackboard()
    snapshot = TelemetrySnapshot(
        heartbeat=Heartbeat(0, 0, mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 0, 0, 3),
        vfr_hud=VfrHud(0., 0., 0, 0, 12.5, 0.),
        heartbeat_at=9.8, vfr_hud_at=9.9,
    )
    board.telemetry = snapshot
    board.view_alive = True
    context = StepContext(board, 10.)

    assert board.telemetry is snapshot
    assert (context.now_s, context.altitude_m, context.altitude_at_s) == (10., 12.5, 9.9)
    assert context.firmware_armed and context.view_alive

    mission = FlightMission(blackboard=board)
    mission.step(context)
    assert mission.phase == Phase.FAULT  # Armed before App's prearm command.


def test_missing_altitude_is_rejected_before_mission_changes():
    board = Blackboard()
    mission = FlightMission(blackboard=board)
    context = StepContext(board, 10.)

    try:
        mission.step(context)
    except ValueError as error:
        assert str(error) == 'Fresh finite MAVLink altitude required'
    else:
        raise AssertionError('Missing altitude was accepted')
    assert mission.phase == Phase.PREARM
    assert mission.entered_at is None
