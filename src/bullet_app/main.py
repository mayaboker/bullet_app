"""Standalone automatic mission entry point."""
import time
import click
from loguru import logger

from .cli import CliParser
from .app import FcuConnectionRetry, run_mission
from .run_logging import configure_logging
from .view_link import RestartRequested


FCU_RETRY_DELAY_S = 1.0


def main(argv=None) -> int:
    try:
        configure_logging()
        options = CliParser().parse(argv)
        if isinstance(options, int):
            return options
        restart_epoch = 0
        retry_fcu = False
        while True:
            try:
                return run_mission(options, restart_epoch=restart_epoch, retry_fcu=retry_fcu)
            except RestartRequested:
                restart_epoch += 1
                retry_fcu = True
                logger.info('View requested a new manual flight session')
            except FcuConnectionRetry as error:
                logger.warning('FCU connection unavailable; retrying in {} s: {}', FCU_RETRY_DELAY_S, error)
                time.sleep(FCU_RETRY_DELAY_S)
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
