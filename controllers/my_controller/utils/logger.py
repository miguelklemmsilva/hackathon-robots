# utils/logger.py
import logging
import os

def setup_logger(name: str, log_file: str, level=logging.INFO):
    """Set up a logger that writes to both a file and the console."""
    if os.path.exists(log_file):
        os.remove(log_file)

    formatter = logging.Formatter('%(asctime)s %(levelname)s: %(message)s')

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
