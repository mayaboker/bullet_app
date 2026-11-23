from math import radians, tan
from types import SimpleNamespace

import pytest
from beta_client.timesync import TimeSyncStatus
from los_tracking.types import ControlDemand, State
from los_tracking.geometry import euler, world_bearing

from bullet_app.blackboard import StepContext
from bullet_app.cli import DEFAULT_TARGET_PROFILE
from bullet_app.flight import Action, Phase
from bullet_app.target import TargetMission, _quaternion
from bullet_app.missions.target_hit import _demand_command
from conftest import tick


def _packet(now, box=None):
    return dict(schema_version=3, image_topic='camera/front/image', session='a' * 32,
                sequence=1, capture_ns=round(now * 1e9), width=640, height=480,
                drone_id='drone_1', bbox=box)


def _sample(now, altitude=10., yaw=0.):
    return SimpleNamespace(
        attitude=SimpleNamespace(roll=0., pitch=0., yaw=yaw, yawspeed=0., time_boot_ms=round(now * 1000)),
        attitude_at=now, vfr_hud=SimpleNamespace(alt=altitude), vfr_hud_at=now)


def _sync(mission, now, offset_ns=0):
    mission.observe_timesync(TimeSyncStatus(offset_ns, 1_000_000, 5, now), now)


def test_target_starts_without_preflight_feed_and_allows_one_attempt():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    assert mission.height_m == 30.0
    tick(mission, 1., 0., 1., False, True)
    tick(mission, 13., 0., 13., False, True)
    _sync(mission, 13.)
    mission.request(Action.ARM, 13.1)
    tick(mission, 13.2, 0., 13.2, True, True)
    mission.request(Action.START, 13.3, 0.)
    assert mission.phase == Phase.TAKEOFF
    with pytest.raises(ValueError, match='one attempt'):
        mission.request(Action.START, 14., 0.)


def test_target_enters_search_after_hover_without_bbox():
    mission = TargetMission(DEFAULT_TARGET_PROFILE, mode='auto')
    mission.phase = Phase.HOVER
    mission.entered_at = 100.
    mission.ground_altitude_m = 0.
    mission.started = True
    for index in range(1, 130):
        now = 100. + index * .1
        mission.blackboard.telemetry = _sample(now, altitude=mission.height_m)
        tick(mission, now, mission.height_m, now, True, False)
        if mission.phase == Phase.SEARCH:
            break
    assert mission.phase == Phase.SEARCH
    assert mission.blackboard.visual.bbox is None
    assert 105. <= now < 106.


def test_target_search_failure_lands_and_contact_disarms():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.SEARCH
    mission.ground_altitude_m = 0.
    mission.height_m = 10.
    mission.search_started = 1.
    mission.blackboard.telemetry = _sample(62.)
    command = tick(mission, 62., 10., 62., True, True)
    assert mission.phase == Phase.LAND and mission.result == 'target not found'
    assert command.arm
    other = TargetMission(DEFAULT_TARGET_PROFILE)
    other.phase = Phase.LOS
    other.ground_altitude_m = 0.
    other.height_m = 10.
    other.blackboard.telemetry = _sample(20.)
    other.observe_simulation_contact({'target_id': 'red_truck', 'drone_id': 'drone_1'})
    command = tick(other, 20., 10., 20., True, True)
    assert other.phase == Phase.DISARMING and other.result == 'hit'
    assert not command.arm and command.throttle == 1000


def test_commit_without_simulation_contact_lands_after_timeout():
    mission = TargetMission(DEFAULT_TARGET_PROFILE, mode='auto')
    mission.phase = Phase.COMMIT
    mission.ground_altitude_m = 0.
    mission.height_m = 10.
    mission.los_started = 100.
    mission.commit_started = 100.
    mission.last_altitude_at = 103.1
    mission.blackboard.telemetry = _sample(103.1)
    mission.blackboard.telemetry.heartbeat = SimpleNamespace(base_mode=128)
    mission._sensor_inputs = lambda _: None
    mission.navigator.update = lambda *_args, **_kwargs: SimpleNamespace(
        state=State.COMMIT, command=None, valid_until_s=0.)
    command = mission._target_step(StepContext(mission.blackboard, 103.1))
    assert mission.phase == Phase.LAND
    assert mission.result == 'contact not confirmed'
    assert command.arm


def test_mavlink_quaternion_is_converted_to_bullet_axes():
    q = _quaternion(0., 0., 0.5)
    assert euler(q)[2] == pytest.approx(0.5)


def test_los_world_bearing_stays_fixed_as_camera_yaws_toward_target():
    camera = TargetMission(DEFAULT_TARGET_PROFILE).config.camera
    target_angle = 0.1
    width = 16
    centered = (camera.cx_px - width / 2, camera.cy_px, width, 20)
    shifted = (camera.cx_px + camera.fx_px * tan(target_angle) - width / 2,
               camera.cy_px, width, 20)
    initial = world_bearing(centered, camera, _quaternion(0., 0., 0.), 0.)[1]
    after_yaw = world_bearing(shifted, camera, _quaternion(0., 0., target_angle), 0.)[1]
    assert after_yaw == pytest.approx(initial, abs=1e-12)


def test_search_sweep_accumulates_observed_positive_yaw():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.SEARCH
    mission._update_search_yaw(_sample(1., yaw=0.2))
    mission._update_search_yaw(_sample(2., yaw=0.5))
    assert mission.swept == pytest.approx(0.3)


def test_los_thrust_mapping_uses_captured_hover_pwm():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    hover_thrust = mission.config.vehicle.mass_kg * mission.config.vehicle.gravity_m_s2
    demand = lambda thrust: ControlDemand(0., 0., 0., thrust)
    assert _demand_command(demand(hover_thrust), mission.config, hover_pwm=1540).throttle == 1540
    assert _demand_command(demand(hover_thrust + 2.), mission.config, hover_pwm=1540).throttle > 1540
    assert _demand_command(demand(hover_thrust - 2.), mission.config, hover_pwm=1540).throttle < 1540
    assert _demand_command(demand(0.), mission.config, hover_pwm=1050).throttle == 1050
    assert _demand_command(demand(mission.config.vehicle.max_thrust_n),
                           mission.config, hover_pwm=1900).throttle == 2000


@pytest.mark.parametrize('angle_deg, expected_pwm', [
    (0, 1500), (10, 1722), (-10, 1278), (45, 2000), (-45, 1000), (50, 2000),
])
def test_los_angle_demand_uses_betaflight_actual_rate_curve(angle_deg, expected_pwm):
    config = TargetMission(DEFAULT_TARGET_PROFILE).config
    hover_thrust = config.vehicle.mass_kg * config.vehicle.gravity_m_s2
    roll = _demand_command(ControlDemand(radians(angle_deg), 0., 0., hover_thrust),
                           config, hover_pwm=1540)
    pitch = _demand_command(ControlDemand(0., radians(angle_deg), 0., hover_thrust),
                            config, hover_pwm=1540)
    assert roll.roll == expected_pwm
    assert pitch.pitch == expected_pwm


def test_settle_to_los_retains_active_throttle_baseline():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.SETTLE
    mission.controller.throttle = 1540
    mission.ground_altitude_m = 0.
    mission._enter(Phase.LOS, 100.)
    assert mission.los_hover_pwm == 1540
    mission.controller.throttle = 1600
    mission.blackboard.telemetry = _sample(100.02)
    mission._sensor_inputs = lambda _: None
    mission.navigator.update = lambda *_args, **_kwargs: SimpleNamespace(
        state=State.TRACK, command=ControlDemand(0., 0., 0.,
            mission.config.vehicle.mass_kg * mission.config.vehicle.gravity_m_s2),
        valid_until_s=100.04)
    command = tick(mission, 100.02, 10., 100.02, True, True)
    assert command.throttle == 1540
    assert mission.los_hover_pwm == 1540


def test_settle_requires_one_second_below_one_degree_per_second():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.CENTER
    mission.search_started = 100.
    mission.ground_altitude_m = 0.
    mission._sensor_inputs = lambda _: None
    mission.navigator.update = lambda *_args, **_kwargs: SimpleNamespace(
        state=State.ACQUIRE, command=None, valid_until_s=0.)

    def step(now, yaw_speed_deg_s):
        mission.blackboard.telemetry = _sample(now, altitude=mission.height_m)
        mission.blackboard.telemetry.attitude.yawspeed = radians(yaw_speed_deg_s)
        mission.observe_bbox(_packet(now, dict(x=312, y=220, width=16, height=20)), now)
        tick(mission, now, mission.height_m, now, True, True)

    step(100., 4.)
    assert mission.phase == Phase.SETTLE
    assert mission.settled_at is None
    step(100.1, 0.8)
    assert mission.settled_at == 100.1
    step(100.9, 0.8)
    assert mission.phase == Phase.SETTLE
    step(101., 1.2)
    assert mission.settled_at is None
    step(101.1, 0.8)
    step(102.09, 0.8)
    assert mission.phase == Phase.SETTLE
    step(102.11, 0.8)
    assert mission.phase == Phase.LOS


def test_los_uses_mavlink_samples_and_bbox_to_command_forward_pitch():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.LOS
    mission.ground_altitude_m = 0.
    mission.height_m = 10.
    mission.los_started = 100.
    command = None
    for index in range(1, 23):
        now = 100. + index * .02
        mission.blackboard.telemetry = _sample(now - .005)
        _sync(mission, now)
        packet = _packet(now - .01, dict(x=300, y=220, width=40, height=40))
        packet['sequence'] = index
        mission.observe_bbox(packet, now)
        command = tick(mission, now, 10., now - .005, True, True)
    assert mission.phase == Phase.LOS
    assert command.arm and command.pitch > 1500
    assert mission.blackboard.features.navigation_input is not None
    assert mission.blackboard.features.navigation_result is not None


def test_attitude_clock_mapping_uses_timesync_offset():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    for index in range(1501):
        now = 100. + index * .02
        sample = _sample(now - .01)
        sample.attitude.time_boot_ms = round(index * 19.6)
        mission.blackboard.telemetry = sample
        offset_ns = round((sample.attitude_at - sample.attitude.time_boot_ms / 1000) * 1e9)
        _sync(mission, now, offset_ns)
        mission._observe_telemetry(now)
    assert mission.sensor_estimator.attitude_latest is not None
    assert now - mission.sensor_estimator.attitude_latest.timestamp_s < .06
    assert mission.timesync_status.offset_ns / 1e9 - 100. > .5


def test_los_lands_when_raw_attitude_source_becomes_stale():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.LOS
    mission.ground_altitude_m = 0.
    mission.height_m = 10.
    mission.los_started = 100.
    mission.blackboard.telemetry = _sample(100.)
    tick(mission, 100., 10., 100., True, True)
    tick(mission, 100.21, 10., 100., True, True)
    assert mission.phase == Phase.LAND
    assert mission.result == 'LOS attitude source stale'


def test_sync_gates_arm_and_faults_after_startup_timeout():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.DISARMED
    mission.observe_timesync(None, 1.)
    assert mission.readiness == 0
    with pytest.raises(ValueError, match='TIMESYNC'):
        mission.request(Action.ARM, 1.1)
    mission.observe_timesync(None, 6.01)
    assert mission.phase == Phase.FAULT
    assert mission.fault_reason == 'MAVLink TIMESYNC unavailable'


def test_sync_loss_disarms_armed_idle_and_lands_airborne():
    idle = TargetMission(DEFAULT_TARGET_PROFILE)
    idle.phase = Phase.ARMED
    _sync(idle, 10.)
    idle.observe_timesync(idle.timesync_status, 11.01)
    assert idle.phase == Phase.DISARMING
    assert idle.result == 'MAVLink TIMESYNC lost'

    airborne = TargetMission(DEFAULT_TARGET_PROFILE)
    airborne.phase = Phase.LOS
    airborne.ground_altitude_m = 0.
    _sync(airborne, 10.)
    airborne.observe_timesync(airborne.timesync_status, 11.01)
    assert airborne.phase == Phase.LAND
    assert airborne.result == 'MAVLink TIMESYNC lost'


def test_attitude_mapping_uses_vehicle_boot_time_not_arrival():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    sample = _sample(100.)
    sample.attitude.time_boot_ms = 5000
    sample.attitude_at = 100.03
    mission.blackboard.telemetry = sample
    _sync(mission, 100.03, 95_000_000_000)
    mission._observe_telemetry(100.03)
    assert mission.sensor_estimator.attitude_latest.timestamp_s == 100.


def test_future_mapped_attitude_uses_fresh_arrival_time():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    sample = _sample(100.)
    sample.attitude.time_boot_ms = 5000
    sample.attitude_at = 100.03
    mission.blackboard.telemetry = sample
    _sync(mission, 100.03, 95_136_000_000)
    mission._observe_telemetry(100.03)
    assert mission.last_attitude_mapped_age_s == pytest.approx(-0.106)
    assert mission.sensor_estimator.attitude_latest.timestamp_s == 100.03


def test_far_future_mapped_attitude_is_rejected():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    sample = _sample(100.)
    sample.attitude.time_boot_ms = 5000
    sample.attitude_at = 100.03
    mission.blackboard.telemetry = sample
    _sync(mission, 100.03, 95_500_000_000)
    mission._observe_telemetry(100.03)
    assert mission.sensor_estimator.attitude_latest is None
    assert mission.attitude_mapping_rejections == 1
