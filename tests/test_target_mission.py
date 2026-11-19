from types import SimpleNamespace

import pytest
from beta_client.timesync import TimeSyncStatus

from bullet_app.cli import DEFAULT_TARGET_PROFILE
from bullet_app.flight import Phase
from bullet_app.target import TargetMission, _quaternion


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
    mission.step(1., 0., 1., False, True)
    mission.step(13., 0., 13., False, True)
    _sync(mission, 13.)
    mission.request('arm', 13.1)
    mission.step(13.2, 0., 13.2, True, True)
    mission.request('start', 13.3, 0.)
    assert mission.phase == Phase.TAKEOFF
    with pytest.raises(ValueError, match='one attempt'):
        mission.request('start', 14., 0.)


def test_target_enters_search_after_hover_without_bbox():
    mission = TargetMission(DEFAULT_TARGET_PROFILE, mode='auto')
    mission.phase = Phase.HOVER
    mission.entered_at = 100.
    mission.ground_altitude_m = 0.
    mission.started = True
    for index in range(1, 130):
        now = 100. + index * .1
        mission.sample = _sample(now, altitude=10.)
        mission.step(now, 10., now, True, False)
        if mission.phase == Phase.SEARCH:
            break
    assert mission.phase == Phase.SEARCH
    assert mission.bbox is None


def test_target_search_failure_lands_and_contact_disarms():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.SEARCH
    mission.ground_altitude_m = 0.
    mission.height_m = 10.
    mission.search_started = 1.
    mission.sample = _sample(62.)
    command = mission.step(62., 10., 62., True, True)
    assert mission.phase == Phase.LAND and mission.result == 'target not found'
    assert command.arm
    other = TargetMission(DEFAULT_TARGET_PROFILE)
    other.phase = Phase.LOS
    other.ground_altitude_m = 0.
    other.height_m = 10.
    other.sample = _sample(20.)
    other.observe_contact({'target_id': 'red_truck', 'drone_id': 'drone_1'})
    command = other.step(20., 10., 20., True, True)
    assert other.phase == Phase.DISARMING and other.result == 'hit'
    assert not command.arm and command.throttle == 1000


def test_mavlink_quaternion_is_converted_to_bullet_axes():
    q = _quaternion(0., 0., 0.5)
    assert q[2] < 0


def test_los_uses_mavlink_samples_and_bbox_to_command_forward_pitch():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.LOS
    mission.ground_altitude_m = 0.
    mission.height_m = 10.
    mission.los_started = 100.
    command = None
    for index in range(1, 23):
        now = 100. + index * .02
        mission.sample = _sample(now - .005)
        _sync(mission, now)
        packet = _packet(now - .01, dict(x=300, y=220, width=40, height=40))
        packet['sequence'] = index
        mission.observe_bbox(packet, now)
        command = mission.step(now, 10., now - .005, True, True)
    assert mission.phase == Phase.LOS
    assert command.arm and command.pitch > 1500


def test_attitude_clock_mapping_uses_timesync_offset():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    for index in range(1501):
        now = 100. + index * .02
        sample = _sample(now - .01)
        sample.attitude.time_boot_ms = round(index * 19.6)
        mission.sample = sample
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
    mission.sample = _sample(100.)
    mission.step(100., 10., 100., True, True)
    mission.step(100.21, 10., 100., True, True)
    assert mission.phase == Phase.LAND
    assert mission.result == 'LOS attitude source stale'


def test_sync_gates_arm_and_faults_after_startup_timeout():
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    mission.phase = Phase.DISARMED
    mission.observe_timesync(None, 1.)
    assert mission.readiness == 0
    with pytest.raises(ValueError, match='TIMESYNC'):
        mission.request('arm', 1.1)
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
    mission.sample = sample
    _sync(mission, 100.03, 95_000_000_000)
    mission._observe_telemetry(100.03)
    assert mission.sensor_estimator.attitude_latest.timestamp_s == 100.
