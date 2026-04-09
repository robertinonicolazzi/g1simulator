#!/bin/sh
# InOrbit agent upstart script. It keeps the agent running and
# automatically performs updates between respawns.
# This is to be replaced by a proper upstart script.

# TODO(adamantivm) Sanity checks of all sorts:
# - That we are in the correct directory (or else go there)
# - That the virtualenv exists
# - That activating the virtualenv was successful
# - That the agent and requirements.txt (full and core) are there

# TODO(adamantivm) Parameterize properly:
# - Virtualenv dir, retry interval

# Calculate INORBIT_DIST based on the directory of the current script
MY_DIR="$(dirname $0)"
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

INORBIT_ENV=${INORBIT_HOME}/local/agent.env.sh
INORBIT_LOG=${INORBIT_HOME}/local/inorbit.log


# Run the agent
run_agent()
{
  cd ${INORBIT_DIST}
  # Activate agent virtualenv
  . ./venv/bin/activate
  # HACK(adamantivm) Set-up LD_LIBRARY_PATH to make sure ROS libs are found by python
  # This should be replaced by a dynamic loading mechanism inside the ros.py agentlet
  SUPPORTED_ROS2_DISTROS="foxy humble iron jazzy"
  UBUNTU_CODE=$(lsb_release -cs)
  # At this point we could assume that the 'INORBIT_ROS' env variable was set at
  # installation time, but if the user installed the agent using Debian, for example,
  # it could not be available.
  if [ `echo ${SUPPORTED_ROS2_DISTROS} | grep -o ${INORBIT_ROS}` ]
  then
    export LD_LIBRARY_PATH=/opt/ros/${INORBIT_ROS}/lib:/opt/ros/${INORBIT_ROS}/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
  else
    echo 2>&1
    echo "INORBIT_ROS env variable not set on local/agent.env.sh." 2>&1
    echo "Using default LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" 2>&1
    export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
  fi
  
  # Agent watchdog
  if [ "${INORBIT_ENABLE_WATCHDOG}" = "yes" ]; then
    WATCHDOG_PID=`pidof -x "agent_watchdog.sh"`
    if [ -z $WATCHDOG_PID ]; then
      echo "Starting agent watchdog" >> ${INORBIT_LOG} 2>&1
      ./scripts/agent_watchdog.sh >> ${INORBIT_HOME}/local/agent_watchdog.log &
    fi
  fi

  # Only log stdout and stderr if the INORBIT_LOG_LEVEL is set to DEBUG, otherwise
  # send it to /dev/null to avoid potential spam from ROS and other lower level
  # libraries
  if [ "${INORBIT_LOG_LEVEL}" = "debug" ]; then
    INORBIT_STDOUT="${INORBIT_LOG}"
  else
    INORBIT_STDOUT="/dev/null"
  fi

  # Start the agent
  python3 -u inorbit.py >> ${INORBIT_STDOUT} 2>&1
}

# In case this is the very first time the agent is started and
# no configuration has been created yet, create local working
# directory now
mkdir -p "${INORBIT_HOME}/local"

# HACK: Very basic log rotation to avoid excesive log file size
# TODO(adamantivm) Implement proper log rotation and centralization
rm "${INORBIT_LOG}.1" > /dev/null 2>&1
mv "${INORBIT_LOG}" "${INORBIT_LOG}.1" > /dev/null 2>&1

# TODO(adamantivm) Unify this with the agent's python-side logging
# (@see inorbit/__init__.py)
AGENT_LOG="${INORBIT_HOME}/local/inorbit_agent.log"
rm "${AGENT_LOG}.1" > /dev/null 2>&1
mv "${AGENT_LOG}" "${AGENT_LOG}.1" > /dev/null 2>&1

# Proactively delete any leftover badly closed module state file, see IO-6890
rm "${INORBIT_HOME}/local/__db..module_states" > /dev/null 2>&1

echo "---------------------------------------" >> ${INORBIT_LOG} 2>&1
echo "InOrbit Agent startup." >> ${INORBIT_LOG} 2>&1
date >> ${INORBIT_LOG} 2>&1
echo "INORBIT_DIST=${INORBIT_DIST}" >> ${INORBIT_LOG} 2>&1
echo "---------------------------------------" >> ${INORBIT_LOG} 2>&1

# Load the environment variables
if [ ! -f ${INORBIT_ENV} ]
then
  echo "Agent configuration file missing. Will keep retrying." >> ${INORBIT_LOG} 2>&1
  while [ ! -f ${INORBIT_ENV} ]
  do
    sleep 5
  done
fi
. ${INORBIT_ENV}

echo "Current configuration:" >> ${INORBIT_LOG} 2>&1
cat ${INORBIT_ENV} >> ${INORBIT_LOG} 2>&1
grep VERSION ${INORBIT_DIST}/inorbit/__init__.py >> ${INORBIT_LOG} 2>&1
grep VARIANT ${INORBIT_DIST}/inorbit/__init__.py >> ${INORBIT_LOG} 2>&1
echo "---------------------------------------" >> ${INORBIT_LOG} 2>&1


run_agent
