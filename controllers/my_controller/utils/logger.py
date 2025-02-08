# utils/logger.py

import logging


def setup_logger(name: str, log_file: str, level=logging.INFO):
    """Set up a logger with a given name and log file."""
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')

    # Create a logger
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # File handler
    fh = logging.FileHandler(log_file)
    fh.setFormatter(formatter)

    # Console handler
    ch = logging.StreamHandler()
    ch.setFormatter(formatter)

    logger.addHandler(fh)
    logger.addHandler(ch)

    return logger

# Example:
# logger = setup_logger('spot', 'spot_autonomous.log')
