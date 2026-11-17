"""Standalone automatic mission entry point."""
import sys

import click
from loguru import logger

from .cli import CliParser
from .mission import run_mission


def main(argv=None) -> int:
    logger.remove()
    logger.add(sys.stderr, level='INFO', format='{message}')
    try:
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
