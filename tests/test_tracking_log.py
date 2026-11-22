"""Persistent logger and bounded target-guidance diagnostic checks."""
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest
from loguru import logger

from bullet_app.missions import Command, Phase
from bullet_app.blackboard import Blackboard, StepContext
from bullet_app.run_logging import configure_logging
from bullet_app.tracking_log import TrackingLog
from los_tracking.types import ControlDemand, Diagnostics, NavigationResult, State, TrackStage


def test_run_log_creates_timestamped_file_and_keeps_debug_out_of_terminal(tmp_path, capsys):
    try:
        first = configure_logging(tmp_path, timestamp=datetime(2026, 11, 19, 12, 0, 0, 123456, timezone.utc))
        logger.debug('file-only checkpoint')
        logger.info('flight event')
        logger.remove()
        assert first.name.startswith('bullet_app_20261119T120000123456+0000_')
        assert first.suffix == '.log'
        contents = first.read_text()
        assert 'file-only checkpoint' in contents and 'flight event' in contents
        assert 'file-only checkpoint' not in capsys.readouterr().err
        second = configure_logging(tmp_path, timestamp=datetime(2026, 11, 19, 12, 0, 0, 123457, timezone.utc))
        assert second != first and second.exists()
    finally:
        logger.remove()


def test_unwritable_log_stops_before_mission(tmp_path, monkeypatch):
    from bullet_app import main as app_main

    blocked = tmp_path / 'not_a_directory'
    blocked.write_text('occupied')
    with pytest.raises(OSError):
        configure_logging(blocked)
    monkeypatch.setattr(app_main, 'configure_logging', lambda: configure_logging(blocked))
    monkeypatch.setattr(app_main, 'run_mission', lambda _: pytest.fail('mission started without a log'))
    try:
        assert app_main.main([]) == 1
    finally:
        logger.remove()


def test_tracking_checkpoints_and_events_are_bounded():
    messages = []
    sink = logger.add(lambda message: messages.append(str(message)), level='DEBUG', format='{message}')
    try:
        packet = {'capture_ns': 1_000_000_000, 'bbox': {'x': 300, 'y': 220, 'width': 40, 'height': 40}}
        mission = SimpleNamespace(
            phase=Phase.SEARCH, result=None, ground_altitude_m=0.,
            config=SimpleNamespace(camera=SimpleNamespace(cx_px=320.)))
        board = Blackboard()
        board.visual.bbox = packet
        mission._visual_box = lambda now: (board.visual.bbox['bbox'] if board.visual.bbox
                                            and now - board.visual.bbox['capture_ns'] / 1e9 <= .2 else None)
        board.telemetry = SimpleNamespace(attitude=SimpleNamespace(yaw=.25), vfr_hud=SimpleNamespace(alt=10.))
        trace = TrackingLog()
        command = Command(1500, True, yaw=1550)
        trace.observe(mission, StepContext(board, 1.), command)
        trace.observe(mission, StepContext(board, 1.1), command)
        board.visual.bbox = {'capture_ns': 1_210_000_000, 'bbox': None}
        trace.observe(mission, StepContext(board, 1.21), command)
        board.visual.contact = True
        mission.result = 'target not found'
        trace.observe(mission, StepContext(board, 1.42), command)
        trace.observe(mission, StepContext(board, 1.43), command)
        checkpoints = [line for line in messages if 'TRACK checkpoint' in line]
        assert len(checkpoints) == 3
        assert 'bbox_error_x_px=0.0' in checkpoints[0]
        assert 'los_state=None' in checkpoints[0]
        assert 'rc_yaw=1550' in checkpoints[0]
        assert sum('event=visible' in line for line in messages) == 1
        assert sum('event=lost' in line for line in messages) == 1
        assert sum('event=contact' in line for line in messages) == 1
        assert sum('event=failure' in line for line in messages) == 1
    finally:
        logger.remove(sink)


def test_los_stage_and_processed_guidance_are_logged():
    messages = []
    sink = logger.add(lambda message: messages.append(str(message)), level='DEBUG', format='{message}')
    try:
        box = {'x': 300, 'y': 220, 'width': 40, 'height': 40}
        mission = SimpleNamespace(
            phase=Phase.LOS, result=None, ground_altitude_m=0.,
            config=SimpleNamespace(camera=SimpleNamespace(cx_px=320.)))
        board = Blackboard()
        board.visual.bbox = {'capture_ns': 2_000_000_000, 'bbox': box}
        mission._visual_box = lambda now: board.visual.bbox['bbox']
        diagnostics = Diagnostics(processed_image_sequence=7,
                                  processed_bbox=(300., 220., 40., 40.),
                                  azimuth_rad=.2, optical_ttc_s=3., vision_reason='usable')
        demand = ControlDemand(.1, .2, .3, 5.)
        board.features.navigation_result = NavigationResult(1, 2., 2.5, State.TRACK,
                                                            TrackStage.BEARING, 'tracking', demand, diagnostics)
        trace = TrackingLog()
        board.telemetry = SimpleNamespace(attitude=SimpleNamespace(yaw=.4), vfr_hud=SimpleNamespace(alt=9.))
        command = Command(1540, True, roll=1510, pitch=1520, yaw=1530)
        trace.observe(mission, StepContext(board, 2.), command)
        trace.observe(mission, StepContext(board, 2.1), command)
        board.features.navigation_result = NavigationResult(2, 2.21, 2.7, State.TRACK,
                                                            TrackStage.TTC, 'tracking', demand, diagnostics)
        trace.observe(mission, StepContext(board, 2.21), command)
        assert sum('LOS stage=bearing' in line for line in messages) == 1
        assert sum('LOS stage=ttc' in line for line in messages) == 1
        checkpoints = [line for line in messages if 'TRACK checkpoint' in line]
        assert len(checkpoints) == 2
        assert 'processed_seq=7' in checkpoints[0]
        assert 'demand_yaw_rate_rad_s=0.3' in checkpoints[0]
        assert 'rc_pitch=1520' in checkpoints[0]
    finally:
        logger.remove(sink)
