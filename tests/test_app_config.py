"""Betaflight TCP and serial configuration before opening connections."""
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
