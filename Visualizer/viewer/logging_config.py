"""Logging configuration for VL53L5CX viewer."""

import logging
import sys

# Create package logger
logger = logging.getLogger("vl53l5cx_viewer")


def setup_logging(level: int = logging.INFO) -> logging.Logger:
    """Configure logging for the viewer application.

    Args:
        level: Logging level (default: INFO)

    Returns:
        Configured logger instance
    """
    logger.setLevel(level)

    # Avoid duplicate handlers if called multiple times
    if logger.handlers:
        return logger

    # Console handler with formatting
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger
