"""Validated Betaflight transport settings loaded from App YAML."""
from dataclasses import dataclass, replace
from enum import Enum
from math import isfinite
from pathlib import Path
import re

import yaml
from bullet_simulation_protocol import validate_endpoint

DEFAULT_CONFIG = Path(__file__).with_name('app.yaml')
DEFAULT_MAVLINK_BAUDRATE = 57600
MAX_PUBLISH_FPS = 50.0
PUBLISH_TOPICS = frozenset((
    'telemetry/heartbeat', 'telemetry/attitude', 'telemetry/vfr_hud',
    'telemetry/gps_raw_int', 'telemetry/global_position_int', 'telemetry/timesync',
    'visual/bbox', 'simulation/target_contact', 'features/navigation_input',
    'features/navigation_result', 'view/alive',
))


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
class SimulationConfig:
    enabled: bool = False
    contact_endpoint: str | None = None


@dataclass(frozen=True, slots=True)
class TelemetryPublisherConfig:
    endpoint: str | None = None
    topics: tuple[tuple[str, float], ...] = ()

    @property
    def enabled(self) -> bool:
        return any(fps > 0 for _, fps in self.topics)


@dataclass(frozen=True, slots=True)
class AppConfig:
    msp: MspConnection
    mavlink: MavlinkConnection
    simulation: SimulationConfig = SimulationConfig()
    telemetry_publisher: TelemetryPublisherConfig = TelemetryPublisherConfig()

    @classmethod
    def from_yaml(cls, path: Path) -> 'AppConfig':
        """Read selected connection settings before any device is opened."""
        data = yaml.safe_load(path.read_text())
        root = _mapping(data, 'config', {'betaflight', 'simulation', 'telemetry_publisher'}, {'betaflight'})
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
        simulation_data = (_mapping(root['simulation'], 'simulation',
                                    {'enabled', 'contact_endpoint'}, {'enabled'})
                           if 'simulation' in root else None)
        simulation = SimulationConfig(**simulation_data) if simulation_data is not None else SimulationConfig()
        _validate_simulation(simulation)
        publisher = _publisher_config(root['telemetry_publisher']) if 'telemetry_publisher' in root else TelemetryPublisherConfig()
        return cls(msp, mavlink, simulation, publisher)

    def with_overrides(self, *, msp_host: str | None = None, msp_port: int | None = None,
                       mavlink_endpoint: str | None = None,
                       contact_endpoint: str | None = None) -> 'AppConfig':
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
        if contact_endpoint is not None and not self.simulation.enabled:
            raise ValueError('--contact-endpoint requires simulation.enabled: true')
        simulation = (replace(self.simulation, contact_endpoint=contact_endpoint)
                      if contact_endpoint is not None else self.simulation)
        _validate_simulation(simulation)
        return AppConfig(msp, mavlink, simulation, self.telemetry_publisher)


def _publisher_config(value) -> TelemetryPublisherConfig:
    data = _mapping(value, 'telemetry_publisher', {'endpoint', 'topics'}, {'endpoint', 'topics'})
    endpoint = data['endpoint']
    match = re.fullmatch(r'tcp://127\.0\.0\.1:([1-9][0-9]{0,4})', endpoint) if isinstance(endpoint, str) else None
    if match is None or int(match[1]) > 65535:
        raise ValueError('telemetry_publisher.endpoint must be a loopback TCP endpoint')
    topics = data['topics']
    if not isinstance(topics, dict) or not topics.keys() <= PUBLISH_TOPICS:
        raise ValueError('telemetry_publisher.topics must map known topics to FPS')
    for topic, fps in topics.items():
        if type(fps) not in (int, float) or not isfinite(fps) or not 0 <= fps <= MAX_PUBLISH_FPS:
            raise ValueError(f'{topic} FPS must be finite and between 0 and {MAX_PUBLISH_FPS}')
    return TelemetryPublisherConfig(endpoint, tuple((topic, float(fps)) for topic, fps in topics.items()))


def _validate_simulation(simulation: SimulationConfig) -> None:
    if type(simulation.enabled) is not bool:
        raise ValueError('simulation.enabled must be true or false')
    if simulation.enabled:
        _nonempty(simulation.contact_endpoint, 'simulation.contact_endpoint')
        validate_endpoint(simulation.contact_endpoint)
    elif simulation.contact_endpoint is not None:
        _nonempty(simulation.contact_endpoint, 'simulation.contact_endpoint')


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
