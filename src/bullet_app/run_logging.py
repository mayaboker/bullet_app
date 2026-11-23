"""Configure one persistent diagnostic log for each Bullet App process."""
from datetime import datetime
from pathlib import Path
import os
import sys

from loguru import logger

LOG_DIR = Path.home() / 'workspaces' / 'bullet_ng' / 'logs'
TERMINAL_FORMAT = '<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | <level>{message}</level>'
FILE_FORMAT = '{time:YYYY-MM-DD HH:mm:ss.SSS ZZ} | {level: <8} | {message}'


def configure_logging(log_dir: Path = LOG_DIR, *, timestamp: datetime | None = None) -> Path:
    """Open the file before flight; an unwritable destination aborts startup."""
    logger.remove()
    logger.add(sys.stderr, level='INFO', format=TERMINAL_FORMAT, colorize=None)
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = timestamp or datetime.now().astimezone()
    path = log_dir / f'bullet_app_{stamp:%Y%m%dT%H%M%S%f%z}_{os.getpid()}.log'
    logger.add(path, level='DEBUG', format=FILE_FORMAT, colorize=False, mode='x')
    logger.info('log file: {}', path)
    return path
