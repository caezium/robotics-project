import logging
import atexit
import os
from logging.handlers import RotatingFileHandler

def get_logger(name=__name__):
    log_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'output', 'runs')
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, 'simulation.log')

    logger = logging.getLogger(name)
    if getattr(logger, '_custom_configured', False):
        return logger
    logger.setLevel(logging.INFO)

    log_formatter = logging.Formatter('[%(levelname)s] %(message)s')

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(log_formatter)
    logger.addHandler(console_handler)

    # File handler with rotation (5MB per file, keep 1 backup)
    file_handler = RotatingFileHandler(log_path, maxBytes=5*1024*1024, backupCount=1)
    file_handler.setFormatter(log_formatter)
    logger.addHandler(file_handler)

    # Ensure logs are flushed and closed on exit
    atexit.register(logging.shutdown)
    logger._custom_configured = True
    return logger 