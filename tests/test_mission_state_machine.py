"""Phase graph and entry actions shared by the mission implementations."""
import pytest

from bullet_app.cli import DEFAULT_TARGET_PROFILE
from bullet_app.flight import Action, FlightMission, Phase
from bullet_app.missions.base import Readiness
from bullet_app.target import TargetMission
from conftest import tick


def test_machine_rejects_unlisted_phase_changes():
    hover = FlightMission()
    target = TargetMission(DEFAULT_TARGET_PROFILE)

    for mission in (hover, target):
        with pytest.raises(ValueError, match='Invalid mission transition'):
            mission._enter(Phase.LOS, 1.0)
        assert mission.phase == Phase.PREARM
        assert mission.entered_at is None

    with pytest.raises(ValueError, match='Invalid mission transition'):
        hover._enter(Phase.SEARCH, 1.0)


def test_reacquiring_hover_preserves_controller_but_landing_resets_it():
    mission = FlightMission()
    for now, phase in enumerate((Phase.DISARMED, Phase.ARMING, Phase.ARMED,
                                 Phase.TAKEOFF, Phase.HOVER), start=1):
        mission._enter(phase, float(now))
    mission.controller.throttle = 1560
    mission.last_altitude_at = 5.0

    mission._enter(Phase.TAKEOFF, 6.0)
    assert mission.controller.throttle == 1560
    assert mission.last_altitude_at == 5.0
    assert mission.entered_at == 6.0

    mission._enter(Phase.LAND, 7.0)
    assert mission.controller.throttle == 1400
    assert mission.last_altitude_at == 0.0
    assert mission.entered_at == 7.0


def test_target_entry_callbacks_initialize_search_and_los():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    for now, phase in enumerate((Phase.DISARMED, Phase.ARMING, Phase.ARMED,
                                 Phase.TAKEOFF, Phase.HOVER), start=1):
        mission._enter(phase, float(now))
    mission.swept = 1.0
    mission.last_yaw = 0.5

    mission._enter(Phase.SEARCH, 6.0)
    assert (mission.search_started, mission.swept, mission.last_yaw) == (6.0, 0.0, None)
    mission._enter(Phase.SETTLE, 7.0)
    mission._enter(Phase.LOS, 8.0)
    assert mission.los_started == 8.0


@pytest.mark.parametrize('phase', (Phase.SEARCH, Phase.CENTER, Phase.SETTLE,
                                   Phase.LOS, Phase.COMMIT))
def test_target_phases_share_flight_safety_and_land_action(phase):
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = phase
    mission.entered_at = 1.0
    mission.ground_altitude_m = 0.0
    assert mission.readiness == 3
    assert mission._flight_command().arm
    mission.request(Action.LAND, 2.0)
    assert mission.phase == Phase.LAND

    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = phase
    mission.entered_at = 1.0
    mission.ground_altitude_m = 0.0
    mission._handle_view_loss(2.0, False)
    assert mission.phase == Phase.LAND


def test_vision_status_is_provided_by_each_concrete_mission():
    hover = FlightMission()
    target = TargetMission(DEFAULT_TARGET_PROFILE)
    assert hover.vision_alive(2.0)
    assert not target.vision_alive(2.0)
    target.blackboard.visual.bbox = {'capture_ns': int(2.0 * 1e9)}
    assert target.vision_alive(2.1)


def test_readiness_enum_preserves_mavlink_values():
    assert tuple(int(value) for value in Readiness) == (0, 1, 2, 3, 4)
    mission = FlightMission()
    assert mission.readiness is Readiness.UNAVAILABLE
    mission.phase = Phase.DISARMED
    assert mission.readiness is Readiness.DISARMED_IDLE
    mission.phase = Phase.ARMED
    assert mission.readiness is Readiness.ARMED_IDLE
    mission.phase = Phase.HOVER
    assert mission.readiness is Readiness.FLYING
    assert float(mission.readiness) == 3.0
    mission.phase = Phase.FAULT
    assert mission.readiness is Readiness.RESET_REQUIRED


def test_fault_transition_uses_machine_and_blocks_restart():
    mission = FlightMission()
    mission.fault('telemetry stale', 2.0)
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'telemetry stale'
    assert mission.entered_at == 2.0
    with pytest.raises(ValueError, match='fault'):
        mission.request(Action.ARM, 3.0)


def test_prearm_and_arm_timeout_safety_transitions():
    already_armed = FlightMission()
    assert not tick(already_armed, 1.0, 0.0, 1.0, True, True).arm
    assert already_armed.phase == Phase.FAULT
    assert already_armed.fault_reason == 'Vehicle armed before App command'

    mission = FlightMission()
    tick(mission, 1.0, 0.0, 1.0, False, True)
    tick(mission, 13.0, 0.0, 13.0, False, True)
    mission.request(Action.ARM, 13.1)
    assert tick(mission, 28.2, 0.0, 28.2, False, True).arm is False
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'Arming not confirmed'


def test_landing_dwell_and_disarm_timeout():
    mission = FlightMission()
    for now, phase in enumerate((Phase.DISARMED, Phase.ARMING, Phase.ARMED,
                                 Phase.TAKEOFF, Phase.LAND), start=1):
        mission._enter(phase, float(now))
    mission.ground_altitude_m = 0.0
    tick(mission, 5.1, 0.0, 5.1, True, True)
    tick(mission, 5.2, 0.0, 5.2, True, True)
    tick(mission, 5.8, 0.0, 5.8, True, True)
    assert tick(mission, 6.3, 0.0, 6.3, True, True).arm is False
    assert mission.phase == Phase.DISARMING

    tick(mission, 16.4, 0.0, 16.4, True, True)
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'Disarm not confirmed'
