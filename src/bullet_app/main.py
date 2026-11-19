"""Standalone automatic mission entry point."""
import click
from loguru import logger

from .cli import CliParser
from .mission import run_mission
from .run_logging import configure_logging


def main(argv=None) -> int:
    try:
        configure_logging()
        options = CliParser().parse(argv)
        return options if isinstance(options, int) else run_mission(options)
    except click.ClickException as error:
        error.show()
        return error.exit_code
    except (click.Abort, KeyboardInterrupt):
        logger.error('Mission interrupted')
        return 130
    except Exception as error:
        logger.error('Mission failed: {}', error)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
