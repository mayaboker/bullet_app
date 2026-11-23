"""Takeoff-only climb-rate behavior for the shared altitude controller."""

from bullet_app.altitude_control import AltitudeControl


def test_takeoff_uses_four_meter_per_second_ceiling_and_tapers_near_height():
    controller = AltitudeControl()
    controller.update(20., 0., 0., landing=False, takeoff=True)
    assert controller.update(20., 0., .1, landing=False, takeoff=True) == 1650

    controller.reset()
    controller.update(20., 19., 0., landing=False, takeoff=True)
    near_target = controller.update(20., 19., .1, landing=False, takeoff=True)
    assert 1400 < near_target < 1650


def test_hover_and_landing_retain_existing_speed_limits():
    controller = AltitudeControl()
    controller.update(20., 0., 0., landing=False)
    hover_command = controller.update(20., 0., .1, landing=False)
    assert hover_command < 1650

    controller.reset()
    controller.update(0., 20., 0., landing=True)
    assert controller.update(0., 20., .1, landing=True) < 1400
