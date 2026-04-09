#!/bin/bash

# Helper script that checks INORBIT's agent status and kills
# it if the socket is stuck in CLOSE_WAIT state for more than
# WAIT_INTERVAL seconds.
#
# Assumptions
# - This is run under the same user as the inorbit agent (inorbit user for Debian installs)
# - The following tools are present: netstat, lsof (optional)

LOOP_INTERVAL=1    # seconds
WAIT_INTERVAL=60   # seconds

MY_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
INORBIT_DIST="$(cd ${MY_DIR}/.. && pwd)"
INORBIT_HOME="$(cd ${INORBIT_DIST}/.. && pwd)"

# When installed as a deb package, the `dist` folder in `INORBIT_DIST`
# ends up being `/usr/local/inorbit` and `INORBIT_HOME` renders as
# `/usr/local`, making subsequent commands to fail. In that case,
# fallback to the HOME folder of the user running the start.sh script. 
if [ $INORBIT_HOME -ef "/usr/local" ]
then
  INORBIT_HOME="$HOME/.inorbit"
fi


echo "InOrbit agent watchdog starting"
date

while true; do
  # Try to get mqtt port number from .mqtt_broker_address file
  MQTT_BROKER_ADDRESS_FILENAME=${INORBIT_HOME}/local/.mqtt_broker_address
  read -r MQTT_BROKER_ADDRESS < ${MQTT_BROKER_ADDRESS_FILENAME}

  # Get the output of all TCP connections to mqtt port
  # and check if there is any instance in CLOSE_WAIT state
  if [[ ! -z ${MQTT_BROKER_ADDRESS} ]] && [[ $(netstat -tn | grep ${MQTT_BROKER_ADDRESS}) =~ "CLOSE_WAIT" ]]; then

    # Wait and try again to confirm the process continues to stay in CLOSE_WAIT state
    sleep ${WAIT_INTERVAL}

    if [[ $(netstat -tn | grep ${MQTT_BROKER_ADDRESS}) =~ "CLOSE_WAIT" ]]; then
      # At this point, we can conclude that the connection is stuck
      # for at least LOOP_INTERVAL

      # Print a status report for logging purposes
      echo "----------------------------------"
      echo "Agent's MQTT broker address: ${MQTT_BROKER_ADDRESS}"
      echo "Socket in CLOSE_WAIT status"

      # Record all commands for clarity
      set -x
      # Timestamp
      date
      # Agent process status
      ps aux | grep inorbit.py
      # Last 10 lines of InOrbit's agent log
      tail -n 10 ${INORBIT_HOME}/local/inorbit_agent.log
      # Number of CLOSE_WAIT connections
      netstat -nap | grep CLOSE_WAIT | wc -l

      echo "Killing InOrbit's agent to force a restart"

      # Get pid based on process status output for inorbit.py
      AGENT_PID=$(ps -ef | grep inorbit.py | grep -v grep | awk '{ print $2 }')
      # Alternative PID search method in case ps doesn't work
      # MQTT_PORT=$(awk -F_ '{print $2}' <<<"$MQTT_BROKER_ADDRESS")
      # AGENT_PID=$(lsof -i:$MQTT_PORT -t)
      kill -9 $AGENT_PID

      set +x
    fi
  fi
  sleep ${LOOP_INTERVAL}
done
