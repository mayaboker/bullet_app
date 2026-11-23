"""The CLI retries only after a validated View restart request."""
from types import SimpleNamespace

import pytest
import bullet_app.main as app_main
from bullet_app import app
from bullet_app.app import FcuConnectionRetry
from bullet_app.view_link import RestartRequested


def test_main_retries_accepted_restart_and_fcu_open_failure(monkeypatch):
    options = SimpleNamespace(mode='manual')
    calls, sleeps = [], []

    def run_mission(received_options, *, restart_epoch, retry_fcu):
        assert received_options is options
        calls.append((restart_epoch, retry_fcu))
        if len(calls) == 1:
            raise RestartRequested
        if len(calls) == 2:
            raise FcuConnectionRetry('FCU offline')
        return 0

    monkeypatch.setattr(app_main, 'configure_logging', lambda: None)
    monkeypatch.setattr(app_main.CliParser, 'parse', lambda *_: options)
    monkeypatch.setattr(app_main, 'run_mission', run_mission)
    monkeypatch.setattr(app_main.time, 'sleep', sleeps.append)

    assert app_main.main([]) == 0
    assert calls == [(0, False), (1, True), (1, True)]
    assert sleeps == [app_main.FCU_RETRY_DELAY_S]


def test_restarted_manual_session_propagates_fcu_open_failure(monkeypatch):
    closed = []

    class View:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            closed.append('view')

    class Future:
        def done(self):
            return True

        def result(self):
            raise ConnectionError('FCU offline')

    class Executor:
        def __init__(self, **_):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_):
            closed.append('executor')

        def submit(self, *_):
            return Future()

    monkeypatch.setattr(app, 'ViewLink', View)
    monkeypatch.setattr(app, 'ThreadPoolExecutor', Executor)
    options = SimpleNamespace(mode='manual')
    config = SimpleNamespace(view_restart_enabled=True)

    with pytest.raises(FcuConnectionRetry, match='FCU offline'):
        app._run_manual(options, config, restart_epoch=1, retry_fcu=True)
    assert closed == ['executor', 'view']
