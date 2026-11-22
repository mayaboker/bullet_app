from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from bullet_app.blackboard import Blackboard, StepContext


@pytest.fixture(autouse=True)
def reset_blackboard():
    Blackboard().reset()


def context(mission, now, altitude=None, altitude_at=None, armed=False, view_alive=True):
    board = mission.blackboard
    if altitude is not None:
        values = vars(board.telemetry).copy()
        values.update(
            vfr_hud=SimpleNamespace(alt=altitude),
            vfr_hud_at=altitude_at,
            heartbeat=SimpleNamespace(base_mode=(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                                                 if armed else 0)),
            heartbeat_at=now,
        )
        board.telemetry = SimpleNamespace(**values)
    board.view_alive = view_alive
    return StepContext(board, now)


def tick(mission, now, altitude, altitude_at, armed, view_alive):
    return mission.step(context(mission, now, altitude, altitude_at, armed, view_alive))
