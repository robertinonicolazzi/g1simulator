# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
from os import environ
from os.path import expanduser

INORBIT_HOME = environ.get("INORBIT_HOME", expanduser("~") + "/.inorbit")
INORBIT_LOG_PATH = INORBIT_HOME + "/local"
INORBIT_LOG_FILE = INORBIT_LOG_PATH + "/inorbit_agent.log"
INORBIT_MODULE_STATES_CACHE_FILE = INORBIT_HOME + "/local" + "/.module_states"
INORBIT_MQTT_BROKER_ADDRESS_FILE = INORBIT_LOG_PATH + "/.mqtt_broker_address"
INORBIT_AGENT_ROSBAGS_PATH = INORBIT_HOME + "/local/rosbags"
INORBIT_ACTIONS_PATH_DEFAULT = INORBIT_HOME + "/local/user_scripts"
INORBIT_CONNECTION_FILE_DEFAULT = INORBIT_LOG_PATH + "/.connection"
VERSION = "4.22.0.ros2"
VARIANT = "ros2"
