"""Validated Betaflight transport settings loaded from App YAML."""
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path

import yaml

DEFAULT_CONFIG = Path(__file__).with_name('app.yaml')
DEFAULT_MAVLINK_BAUDRATE = 57600


class ConnectionType(Enum):
    TCP = 'tcp'
    SERIAL = 'serial'


@dataclass(frozen=True, slots=True)
class MspConnection:
    type: ConnectionType
    host: str | None = None
    port: int | None = None
    device: str | None = None
    baudrate: int | None = None


@dataclass(frozen=True, slots=True)
class MavlinkConnection:
    type: ConnectionType
    endpoint: str
    baudrate: int


@dataclass(frozen=True, slots=True)
class AppConfig:
    msp: MspConnection
    mavlink: MavlinkConnection

    @classmethod
    def from_yaml(cls, path: Path) -> 'AppConfig':
        """Read selected connection settings before any device is opened."""
        data = yaml.safe_load(path.read_text())
        root = _mapping(data, 'config', {'betaflight'}, {'betaflight'})
        betaflight = _mapping(root['betaflight'], 'betaflight', {'msp', 'mavlink'}, {'msp', 'mavlink'})
        msp_data = _mapping(betaflight['msp'], 'betaflight.msp',
                            {'type', 'host', 'port', 'device', 'baudrate'}, {'type'})
        mavlink_data = _mapping(betaflight['mavlink'], 'betaflight.mavlink',
                                {'type', 'endpoint', 'baudrate'}, {'type', 'endpoint'})

        msp_type = _connection_type(msp_data['type'], 'betaflight.msp.type')
        mavlink_type = _connection_type(mavlink_data['type'], 'betaflight.mavlink.type')
        msp = MspConnection(msp_type, msp_data.get('host'), msp_data.get('port'),
                            msp_data.get('device'), msp_data.get('baudrate'))
        mavlink = MavlinkConnection(mavlink_type, mavlink_data['endpoint'],
                                    mavlink_data.get('baudrate', DEFAULT_MAVLINK_BAUDRATE))
        _validate_msp(msp)
        _validate_mavlink(mavlink)
        return cls(msp, mavlink)

    def with_overrides(self, *, msp_host: str | None = None, msp_port: int | None = None,
                       mavlink_endpoint: str | None = None) -> 'AppConfig':
        """Apply explicit CLI endpoints after loading and validating YAML."""
        if self.msp.type is ConnectionType.SERIAL and (msp_host is not None or msp_port is not None):
            raise ValueError('--msp-host and --msp-port require TCP MSP')
        msp = replace(self.msp,
                      host=self.msp.host if msp_host is None else msp_host,
                      port=self.msp.port if msp_port is None else msp_port)
        mavlink = replace(self.mavlink,
                          endpoint=self.mavlink.endpoint if mavlink_endpoint is None else mavlink_endpoint)
        _validate_msp(msp)
        _validate_mavlink(mavlink)
        return AppConfig(msp, mavlink)


def _mapping(value, name: str, allowed: set[str], required: set[str]) -> dict:
    if not isinstance(value, dict) or not required <= value.keys() or not value.keys() <= allowed:
        raise ValueError(f'{name} requires {sorted(required)} and permits only {sorted(allowed)}')
    return value


def _connection_type(value, name: str) -> ConnectionType:
    try:
        return ConnectionType(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} must be tcp or serial') from error


def _nonempty(value, name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{name} must be a nonempty string')


def _positive_int(value, name: str, *, maximum: int | None = None) -> None:
    if type(value) is not int or value <= 0 or (maximum is not None and value > maximum):
        raise ValueError(f'{name} must be a positive integer' + (f' up to {maximum}' if maximum else ''))


def _validate_msp(msp: MspConnection) -> None:
    if msp.type is ConnectionType.TCP:
        _nonempty(msp.host, 'betaflight.msp.host')
        _positive_int(msp.port, 'betaflight.msp.port', maximum=65535)
    else:
        _nonempty(msp.device, 'betaflight.msp.device')
        _positive_int(msp.baudrate, 'betaflight.msp.baudrate')


def _validate_mavlink(mavlink: MavlinkConnection) -> None:
    _nonempty(mavlink.endpoint, 'betaflight.mavlink.endpoint')
    _positive_int(mavlink.baudrate, 'betaflight.mavlink.baudrate')
    if mavlink.type is ConnectionType.TCP and not mavlink.endpoint.startswith('tcp:'):
        raise ValueError('betaflight.mavlink.endpoint must start with tcp: for TCP')
    if mavlink.type is ConnectionType.SERIAL and mavlink.endpoint.startswith('tcp:'):
        raise ValueError('betaflight.mavlink.endpoint must be a device for serial')
