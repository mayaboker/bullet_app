"""Betaflight TCP and serial configuration before opening connections."""
from contextlib import ExitStack
from types import SimpleNamespace

import pytest

from bullet_app import app
from bullet_app.cli import CliParser
from bullet_app.config import AppConfig, ConnectionType, DEFAULT_CONFIG


SERIAL_CONFIG = """betaflight:
  msp:
    type: serial
    device: /dev/ttyUSB0
    baudrate: 115200
  mavlink:
    type: serial
    endpoint: /dev/ttyUSB1
    baudrate: 57600
"""


def test_packaged_config_keeps_existing_tcp_connections():
    options = CliParser().parse([])
    assert options.config == DEFAULT_CONFIG
    config = AppConfig.from_yaml(options.config)
    assert (config.msp.type, config.msp.host, config.msp.port) == (ConnectionType.TCP, '127.0.0.1', 5761)
    assert (config.mavlink.type, config.mavlink.endpoint, config.mavlink.baudrate) == (
        ConnectionType.TCP, 'tcp:127.0.0.1:5762', 57600)
    assert not config.simulation.enabled


def test_sitl_config_enables_contact_and_cli_override_requires_yaml_gate():
    sitl = AppConfig.from_yaml(DEFAULT_CONFIG.with_name('app.sitl.yaml'))
    assert sitl.simulation.enabled
    assert sitl.simulation.contact_endpoint == 'tcp://127.0.0.1:5555'
    assert sitl.with_overrides(contact_endpoint='tcp://127.0.0.1:6000').simulation.contact_endpoint == (
        'tcp://127.0.0.1:6000')
    with pytest.raises(ValueError, match='requires simulation.enabled'):
        AppConfig.from_yaml(DEFAULT_CONFIG).with_overrides(contact_endpoint='tcp://127.0.0.1:6000')


def test_disabled_simulation_opens_tracker_without_loading_simulation(monkeypatch):
    events = []

    class Tracker:
        def __init__(self, endpoint):
            events.append(('tracker', endpoint))

        def __enter__(self):
            return self

        def __exit__(self, *_):
            events.append('tracker closed')

    monkeypatch.setattr(app, 'TrackerSubscriber', Tracker)
    options = SimpleNamespace(mission='target-hit', bbox_endpoint='tcp://127.0.0.1:5560')
    with ExitStack() as resources:
        tracker, simulation = app._open_target_inputs(options, AppConfig.from_yaml(DEFAULT_CONFIG), resources)
        assert isinstance(tracker, Tracker) and simulation is None
        assert events == [('tracker', 'tcp://127.0.0.1:5560')]
    assert events[-1] == 'tracker closed'


def test_sitl_opens_contact_only_for_target_hit(monkeypatch):
    from bullet_app import simulation

    events = []

    class Subscriber:
        def __init__(self, endpoint):
            events.append(endpoint)

        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

    monkeypatch.setattr(app, 'TrackerSubscriber', Subscriber)
    monkeypatch.setattr(simulation, 'SimulationContactSubscriber', Subscriber)
    config = AppConfig.from_yaml(DEFAULT_CONFIG.with_name('app.sitl.yaml'))
    with ExitStack() as resources:
        assert app._open_target_inputs(SimpleNamespace(mission='hover-land'), config, resources) == (None, None)
        assert events == []
        tracker, contact = app._open_target_inputs(
            SimpleNamespace(mission='target-hit', bbox_endpoint='tcp://127.0.0.1:5560'),
            config, resources)
        assert isinstance(tracker, Subscriber) and isinstance(contact, Subscriber)
    assert events == ['tcp://127.0.0.1:5560', 'tcp://127.0.0.1:5555']


@pytest.mark.parametrize('settings', ('enabled: yes', 'enabled: true',
                                      'enabled: true\n  contact_endpoint: bad'))
def test_simulation_config_rejects_invalid_settings(tmp_path, settings):
    path = tmp_path / 'app.yaml'
    path.write_text(SERIAL_CONFIG + 'simulation:\n  ' + settings + '\n')
    with pytest.raises(ValueError):
        AppConfig.from_yaml(path)


def test_serial_config_selects_each_betaflight_link(tmp_path, monkeypatch):
    path = tmp_path / 'app.yaml'
    path.write_text(SERIAL_CONFIG)
    options = CliParser().parse(['--config', str(path)])
    config = AppConfig.from_yaml(options.config)
    assert config.msp.type is ConnectionType.SERIAL
    assert (config.msp.device, config.msp.baudrate) == ('/dev/ttyUSB0', 115200)
    assert (config.mavlink.endpoint, config.mavlink.baudrate) == ('/dev/ttyUSB1', 57600)

    calls = []
    monkeypatch.setattr(app, 'TcpTransport', lambda *args: calls.append(('tcp', args)))
    monkeypatch.setattr(app, 'SerialTransport', lambda *args: calls.append(('serial', args)))
    app._open_msp(config)
    app._open_msp(AppConfig.from_yaml(DEFAULT_CONFIG))
    assert calls == [('serial', ('/dev/ttyUSB0', 115200)), ('tcp', ('127.0.0.1', 5761))]


def test_cli_overrides_tcp_and_rejects_tcp_flags_for_serial(tmp_path):
    tcp = AppConfig.from_yaml(DEFAULT_CONFIG)
    overridden = tcp.with_overrides(msp_host='localhost', msp_port=6001,
                                    mavlink_endpoint='tcp:localhost:6002')
    assert (overridden.msp.host, overridden.msp.port, overridden.mavlink.endpoint) == (
        'localhost', 6001, 'tcp:localhost:6002')

    path = tmp_path / 'app.yaml'
    path.write_text(SERIAL_CONFIG)
    serial = AppConfig.from_yaml(path)
    with pytest.raises(ValueError, match='require TCP MSP'):
        serial.with_overrides(msp_host='localhost')
    assert serial.with_overrides(mavlink_endpoint='/dev/ttyACM0').mavlink.endpoint == '/dev/ttyACM0'


@pytest.mark.parametrize(('config_text', 'error'), (
    (SERIAL_CONFIG.replace('type: serial', 'type: udp', 1), 'must be tcp or serial'),
    (SERIAL_CONFIG.replace('device: /dev/ttyUSB0', 'device: ""'), 'msp.device'),
    (SERIAL_CONFIG.replace('baudrate: 115200', 'baudrate: 0'), 'msp.baudrate'),
    (SERIAL_CONFIG.replace('endpoint: /dev/ttyUSB1', 'endpoint: tcp:localhost:5762'), 'device for serial'),
))
def test_invalid_connection_config_fails_before_open(tmp_path, config_text, error):
    path = tmp_path / 'app.yaml'
    path.write_text(config_text)
    with pytest.raises(ValueError, match=error):
        AppConfig.from_yaml(path)
