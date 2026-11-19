from bullet_app.rate_monitor import RateMonitor


def test_rate_alerts_ignore_startup_and_debounce_both_streams():
    monitor = RateMonitor(100.)
    assert monitor.check(101., 0., 0.) == ()
    assert monitor.check(102., 39., 50.) == ()
    assert monitor.check(103., 38., 35.) == (('ATTITUDE', 'low', 38.),)
    assert monitor.check(104., 44., 34.) == (('VFR_HUD', 'low', 34.),)
    assert monitor.check(105., 46., 46.) == ()
    assert monitor.check(106., 47., 47.) == (('ATTITUDE', 'recovered', 47.),
                                           ('VFR_HUD', 'recovered', 47.))
    assert monitor.check(106.5, 0., 0.) == ()
