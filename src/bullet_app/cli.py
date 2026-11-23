"""Command-line options for the standalone simulated flight mission."""
from dataclasses import dataclass
from pathlib import Path

import click

from .config import DEFAULT_CONFIG

DEFAULT_MODE = 'manual'
DEFAULT_TARGET_PROFILE = Path(__file__).resolve().parents[3] / 'los_tracking/examples/bullet_profile.yaml'


@dataclass(frozen=True, slots=True)
class AppOptions:
    msp_host: str | None = None
    msp_port: int | None = None
    mavlink_endpoint: str | None = None
    mode: str = DEFAULT_MODE
    mission: str = 'hover-land'
    mission_config: Path | None = None
    bbox_endpoint: str = 'tcp://127.0.0.1:5560'
    contact_endpoint: str | None = None
    config: Path = DEFAULT_CONFIG


class CliParser:
    """Parse options without opening simulation connections."""

    def parse(self, argv=None) -> AppOptions | int:
        @click.command()
        @click.option('--config', type=click.Path(exists=True, dir_okay=False, path_type=Path),
                      default=DEFAULT_CONFIG, show_default=True,
                      help='Betaflight connections and optional simulator contact YAML')
        @click.option('--msp-host', help='Override TCP MSP host from the config')
        @click.option('--msp-port', type=click.IntRange(1, 65535), help='Override TCP MSP port from the config')
        @click.option('--mavlink-endpoint', help='Override MAVLink TCP endpoint or serial device from the config')
        @click.option('--mode', type=click.Choice(('manual', 'auto')), default=DEFAULT_MODE, show_default=True,
                      help='manual waits for View Arm/Start; auto starts one flight without View')
        @click.option('--mission', type=click.Choice(('hover-land', 'target-hit')), default='hover-land', show_default=True)
        @click.option('--mission-config', type=click.Path(exists=True, dir_okay=False, path_type=Path),
                      help='Profile for the selected mission; defaults to its packaged profile')
        @click.option('--bbox-endpoint', default='tcp://127.0.0.1:5560')
        @click.option('--contact-endpoint', help='Override simulator contact endpoint when simulation is enabled')
        def command(config, msp_host, msp_port, mavlink_endpoint, mode, mission, mission_config,
                    bbox_endpoint, contact_endpoint):
            """Run a selected Betaflight mission with manual or automatic approval."""
            return AppOptions(msp_host=msp_host, msp_port=msp_port, mavlink_endpoint=mavlink_endpoint,
                              mode=mode, mission=mission, mission_config=mission_config,
                              bbox_endpoint=bbox_endpoint, contact_endpoint=contact_endpoint, config=config)

        return command.main(args=argv, prog_name='bullet-app', standalone_mode=False)
