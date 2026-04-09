#/bin/sh
echo
echo "Shutting down and completely removing inOrbit Agent from your system."
echo

# Get the location of the current script
# https://stackoverflow.com/questions/59895/get-the-source-directory-of-a-bash-script-from-within-the-script-itself
MY_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
INORBIT_DIST="$(cd ${MY_DIR}/.. && pwd)"
INORBIT_HOME="$(cd ${INORBIT_DIST}/.. && pwd)"

# Stop inorbit agent through the appropriate init system
if [ -e /etc/systemd/system/inorbit.service ]
then
  sudo systemctl stop inorbit
  sudo systemctl disable inorbit
  sudo rm /etc/systemd/system/inorbit.service
elif [ -e /etc/init/inorbit.conf ]
then
  sudo initctl stop inorbit
  sudo rm /etc/init/inorbit.conf
fi

echo "Deleting agent files: ${INORBIT_HOME}"
rm -r ${INORBIT_HOME}
