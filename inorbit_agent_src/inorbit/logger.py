#!/usr/bin/env python
#
# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
#
# InOrbit logging functions
import logging.handlers
import os.path

from inorbit import INORBIT_HOME

INORBIT_LOG_PATH = os.path.join(INORBIT_HOME, "local")
INORBIT_LOG_FILENAME = "inorbit_agent.log"
INORBIT_LOG_FILE = os.path.join(INORBIT_LOG_PATH, INORBIT_LOG_FILENAME)

LOG_FILE_SIZE = 1000000  # 1 Mb
LOG_FILES_COUNT = 9  # Current file plus N older logs

# Set overall logging level to INFO
# Possible log levels
LOG_LEVELS = {
    "debug": logging.DEBUG,
    "info": logging.INFO,
    "warning": logging.WARNING,
    "error": logging.ERROR,
    "critical": logging.CRITICAL,
}

# Set overall logging level to INFO by default
level = "info"

# Check if the user has configured a different log level
if "INORBIT_LOG_LEVEL" in os.environ:
    try:
        level = (os.environ["INORBIT_LOG_LEVEL"]).lower()
    except Exception as e:
        print("Unable to set custom logging level")

logging.basicConfig(level=LOG_LEVELS.get(level, logging.INFO))

# create a file handler
handler = logging.handlers.RotatingFileHandler(
    INORBIT_LOG_FILE, "a", LOG_FILE_SIZE, LOG_FILES_COUNT
)
# ...and a logging format
formatter = logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s")
handler.setFormatter(formatter)


def getLog(name):
    """
    Builds (or gets) a logger for a given module.
    It adds the appropriate handlers to this logging is sent to InOrbit's log
    files.
    """

    # create a new logger
    logger = logging.getLogger(name)
    # Disable propagating to root logger's handler (console)
    logger.propagate = False
    # add the handlers to the new logger
    logger.addHandler(handler)
    logger.setLevel(LOG_LEVELS.get(level, logging.INFO))
    return logger
