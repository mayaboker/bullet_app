from math import cos, sin

from bullet_app.cli import DEFAULT_TARGET_PROFILE
from bullet_app.sensor_estimator import MAX_PREDICTION_S, TargetSensorEstimator
from los_tracking import LosNavigator
from los_tracking.config import NavigatorConfig
from los_tracking.geometry import euler
from los_tracking.types import AttitudeSample, BarometerSample, NavigationInput, State, TargetObservation


def _attitude(sequence, stamp, yaw=0.0):
    return AttitudeSample('raw-attitude', sequence, stamp,
                          (0.0, 0.0, sin(yaw / 2), cos(yaw / 2)))


def _barometer(sequence, stamp, altitude=10.0):
    return BarometerSample('raw-altitude', sequence, stamp, altitude)


def _target(sequence, stamp):
    return TargetObservation('camera', sequence, stamp, (300.0, 220.0, 40.0, 40.0))


def test_fixed_tick_predictions_and_one_delivery_per_camera_frame():
    config = NavigatorConfig.from_yaml(DEFAULT_TARGET_PROFILE)
    estimator = TargetSensorEstimator(config.tuning)
    navigator = LosNavigator(config)
    start = 100.0
    next_sensor = next_camera = start
    sensor_index = camera_index = 0
    delivered = []
    estimated_times = []
    states = []
    for tick_index in range(151):
        now = start + tick_index * .02 + (0.003 if tick_index % 7 == 0 else 0.0)
        while next_sensor + .03 <= now:
            estimator.push_attitude(_attitude(sensor_index, next_sensor))
            estimator.push_barometer(_barometer(sensor_index, next_sensor))
            sensor_index += 1
            next_sensor = start + sensor_index / 47
        while next_camera + .025 <= now:
            estimator.push_target(_target(camera_index, next_camera))
            camera_index += 1
            next_camera = start + camera_index / 30
        inputs = estimator.tick(now)
        if inputs.attitude is not None:
            estimated_times.append(inputs.attitude.timestamp_s)
        if inputs.target is not None:
            delivered.append(inputs.target.sequence)
        result = navigator.update(inputs, now_s=now, enabled=True, command_horizon_s=.02)
        if now >= start + .5:
            states.append(result.state)
    assert delivered == sorted(set(delivered))
    assert len(delivered) >= 75
    assert estimator.overflow_boxes == 0
    assert all(round((new - old) / .02) >= 1 and
               abs((new - old) / .02 - round((new - old) / .02)) < 1e-6
               for old, new in zip(estimated_times, estimated_times[1:]))
    assert result.diagnostics.alignment_replacements == 0
    assert State.TRACK in states
    assert all(state == State.TRACK for state in states[-50:])


def test_prediction_expires_and_missed_ticks_are_counted():
    config = NavigatorConfig.from_yaml(DEFAULT_TARGET_PROFILE)
    estimator = TargetSensorEstimator(config.tuning)
    estimator.push_attitude(_attitude(1, 10.0))
    estimator.push_barometer(_barometer(1, 10.0))
    first = estimator.tick(10.0)
    assert first.attitude is not None and first.barometer is not None
    late = estimator.tick(10.04)
    assert late.attitude is not None and estimator.skipped_ticks == 1
    stale = estimator.tick(10.0 + MAX_PREDICTION_S + .02)
    assert stale.attitude is None and stale.barometer is None


def test_fresh_47_hz_barometer_does_not_go_stale_on_fixed_tick_phase():
    config = NavigatorConfig.from_yaml(DEFAULT_TARGET_PROFILE)
    estimator = TargetSensorEstimator(config.tuning)
    navigator = LosNavigator(config)
    for index in range(1, 47 * 3):
        now = 100. + index / 47 + .0102
        sensor_index = int((now - 100. - .01) * 47)
        sensor_time = 100. + sensor_index / 47 + .01
        estimator.push_barometer(_barometer(sensor_index, sensor_time))
        estimate = estimator.tick(now)
        attitude = _attitude(index, now)
        result = navigator.update(NavigationInput(None, estimate.barometer, attitude),
                                  now_s=now, enabled=True)
        assert result.state != State.FAULT, (result.reason, now - sensor_time)
    assert estimator.barometer_interpolations > 0


def test_measurement_ahead_of_tick_is_interpolated_from_previous_sample():
    config = NavigatorConfig.from_yaml(DEFAULT_TARGET_PROFILE)
    estimator = TargetSensorEstimator(config.tuning)
    estimator.push_attitude(_attitude(1, 10.0))
    estimator.push_barometer(_barometer(1, 10.0, 10.0))
    estimator.tick(10.0)
    estimator.push_attitude(_attitude(2, 10.03, yaw=.3))
    estimator.push_barometer(_barometer(2, 10.03, 12.0))
    estimate = estimator.tick(10.039)
    assert estimate.attitude is not None and estimate.attitude.timestamp_s == 10.02
    assert estimate.barometer is not None and estimate.barometer.timestamp_s == 10.02
    assert 10.0 < estimate.barometer.altitude_m < 12.0
    assert 0.0 < euler(estimate.attitude.orientation_xyzw)[2] < .3
    assert estimator.attitude_interpolations == 1
    assert estimator.barometer_interpolations == 1
    assert estimator.barometer_missing_ticks == 0


def test_attitude_smoothing_and_short_prediction():
    config = NavigatorConfig.from_yaml(DEFAULT_TARGET_PROFILE)
    estimator = TargetSensorEstimator(config.tuning)
    estimator.push_attitude(_attitude(1, 10.0))
    estimator.push_barometer(_barometer(1, 10.0))
    estimator.tick(10.0)
    estimator.push_attitude(_attitude(2, 10.02, yaw=.1))
    estimator.push_barometer(_barometer(2, 10.02))
    smoothed = estimator.tick(10.02)
    predicted = estimator.tick(10.04)
    smooth_yaw = euler(smoothed.attitude.orientation_xyzw)[2]
    predicted_yaw = euler(predicted.attitude.orientation_xyzw)[2]
    assert 0 < smooth_yaw < .1
    assert smooth_yaw < predicted_yaw < .1


def test_old_and_reordered_camera_samples_do_not_refresh_los():
    config = NavigatorConfig.from_yaml(DEFAULT_TARGET_PROFILE)
    estimator = TargetSensorEstimator(config.tuning)
    estimator.push_target(_target(2, 10.0))
    estimator.push_target(_target(1, 9.99))
    estimator.push_attitude(_attitude(1, 10.0))
    estimator.push_barometer(_barometer(1, 10.0))
    first = estimator.tick(10.0)
    assert first.target is not None and first.target.sequence == 2
    assert estimator.reordered_samples == 1
    estimator.push_target(_target(3, 10.01))
    estimator.push_attitude(_attitude(2, 10.02))
    estimator.push_barometer(_barometer(2, 10.02))
    second = estimator.tick(10.02)
    assert second.target is not None and second.target.sequence == 3
    assert estimator.tick(10.04).target is None


def test_expired_box_is_dropped_without_repeating_a_detection():
    config = NavigatorConfig.from_yaml(DEFAULT_TARGET_PROFILE)
    estimator = TargetSensorEstimator(config.tuning)
    estimator.push_target(_target(1, 10.0))
    estimator.push_attitude(_attitude(1, 10.21))
    estimator.push_barometer(_barometer(1, 10.21))
    inputs = estimator.tick(10.21)
    assert inputs.target is None
    assert estimator.expired_boxes == 1
    assert estimator.queue_depth == 0
