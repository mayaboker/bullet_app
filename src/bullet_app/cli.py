"""Command-line options for the standalone simulated flight mission."""
from dataclasses import dataclass
from pathlib import Path

import click

DEFAULT_MSP_HOST = '127.0.0.1'
DEFAULT_MSP_PORT = 5761
DEFAULT_MAVLINK_ENDPOINT = 'tcp:127.0.0.1:5762'
DEFAULT_MODE = 'auto'
DEFAULT_MISSION_CONFIG = Path(__file__).resolve().parents[3] / 'los_tracking/examples/bullet_profile.yaml'


@dataclass(frozen=True, slots=True)
class AppOptions:
    msp_host: str = DEFAULT_MSP_HOST
    msp_port: int = DEFAULT_MSP_PORT
    mavlink_endpoint: str = DEFAULT_MAVLINK_ENDPOINT
    mode: str = DEFAULT_MODE
    mission: str = 'hover-land'
    mission_config: Path = DEFAULT_MISSION_CONFIG
    bbox_endpoint: str = 'tcp://127.0.0.1:5560'
    contact_endpoint: str = 'tcp://127.0.0.1:5555'


class CliParser:
    """Parse options without opening simulation connections."""

    def parse(self, argv=None) -> AppOptions | int:
        @click.command()
        @click.option('--msp-host', default=DEFAULT_MSP_HOST, show_default=True)
        @click.option('--msp-port', type=click.IntRange(1, 65535), default=DEFAULT_MSP_PORT, show_default=True)
        @click.option('--mavlink-endpoint', default=DEFAULT_MAVLINK_ENDPOINT, show_default=True)
        @click.option('--mode', type=click.Choice(('auto', 'manual')), default=DEFAULT_MODE, show_default=True)
        @click.option('--mission', type=click.Choice(('hover-land', 'target-hit')), default='hover-land', show_default=True)
        @click.option('--mission-config', type=click.Path(exists=True, dir_okay=False, path_type=Path), default=DEFAULT_MISSION_CONFIG)
        @click.option('--bbox-endpoint', default='tcp://127.0.0.1:5560')
        @click.option('--contact-endpoint', default='tcp://127.0.0.1:5555')
        def command(msp_host, msp_port, mavlink_endpoint, mode, mission, mission_config, bbox_endpoint, contact_endpoint):
            """Wait for Bullet View to arm and start a SITL flight."""
            if mode == 'manual' and mission != 'hover-land':
                raise click.UsageError('target-hit requires --mode auto')
            return AppOptions(msp_host, msp_port, mavlink_endpoint, mode, mission, mission_config,
                              bbox_endpoint, contact_endpoint)

        return command.main(args=argv, prog_name='bullet-app', standalone_mode=False)
