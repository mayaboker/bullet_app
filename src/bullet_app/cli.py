"""Command-line options for the standalone simulated flight mission."""
from dataclasses import dataclass

import click

DEFAULT_MSP_HOST = '127.0.0.1'
DEFAULT_MSP_PORT = 5761
DEFAULT_MAVLINK_ENDPOINT = 'tcp:127.0.0.1:5762'
DEFAULT_MODE = 'auto'


@dataclass(frozen=True, slots=True)
class AppOptions:
    msp_host: str = DEFAULT_MSP_HOST
    msp_port: int = DEFAULT_MSP_PORT
    mavlink_endpoint: str = DEFAULT_MAVLINK_ENDPOINT
    mode: str = DEFAULT_MODE


class CliParser:
    """Parse options without opening simulation connections."""

    def parse(self, argv=None) -> AppOptions | int:
        @click.command()
        @click.option('--msp-host', default=DEFAULT_MSP_HOST, show_default=True)
        @click.option('--msp-port', type=click.IntRange(1, 65535), default=DEFAULT_MSP_PORT, show_default=True)
        @click.option('--mavlink-endpoint', default=DEFAULT_MAVLINK_ENDPOINT, show_default=True)
        @click.option('--mode', type=click.Choice(('auto', 'manual')), default=DEFAULT_MODE, show_default=True)
        def command(msp_host, msp_port, mavlink_endpoint, mode):
            """Wait for Bullet View to arm and start a SITL flight."""
            return AppOptions(msp_host, msp_port, mavlink_endpoint, mode)

        return command.main(args=argv, prog_name='bullet-app', standalone_mode=False)
