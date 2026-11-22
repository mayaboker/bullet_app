"""Phase graph and entry actions shared by the mission implementations."""
import pytest

from bullet_app.cli import DEFAULT_TARGET_PROFILE
from bullet_app.flight import FlightMission, Phase
from bullet_app.target import TargetMission


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


def test_fault_transition_uses_machine_and_blocks_restart():
    mission = FlightMission()
    mission.fault('telemetry stale', 2.0)
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'telemetry stale'
    assert mission.entered_at == 2.0
    with pytest.raises(ValueError, match='fault'):
        mission.request('arm', 3.0)


def test_prearm_and_arm_timeout_safety_transitions():
    already_armed = FlightMission()
    assert not already_armed.step(1.0, 0.0, 1.0, True, True).arm
    assert already_armed.phase == Phase.FAULT
    assert already_armed.fault_reason == 'Vehicle armed before App command'

    mission = FlightMission()
    mission.step(1.0, 0.0, 1.0, False, True)
    mission.step(13.0, 0.0, 13.0, False, True)
    mission.request('arm', 13.1)
    assert mission.step(28.2, 0.0, 28.2, False, True).arm is False
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'Arming not confirmed'


def test_landing_dwell_and_disarm_timeout():
    mission = FlightMission()
    for now, phase in enumerate((Phase.DISARMED, Phase.ARMING, Phase.ARMED,
                                 Phase.TAKEOFF, Phase.LAND), start=1):
        mission._enter(phase, float(now))
    mission.ground_altitude_m = 0.0
    mission.step(5.1, 0.0, 5.1, True, True)
    mission.step(5.2, 0.0, 5.2, True, True)
    mission.step(5.8, 0.0, 5.8, True, True)
    assert mission.step(6.3, 0.0, 6.3, True, True).arm is False
    assert mission.phase == Phase.DISARMING

    mission.step(16.4, 0.0, 16.4, True, True)
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'Disarm not confirmed'
