#!/usr/bin/env bash
set -euo pipefail

# Source ROS 2 and workspace environment if present, then exec the given command.
# Some ROS setup scripts expect unset variables; temporarily disable nounset around sourcing.
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  # ensure variables referenced by setup scripts exist
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if [ -f /opt/interbotix_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  # ensure variables referenced by setup scripts exist
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  set +u
  source /opt/interbotix_ws/install/setup.bash
  set -u
fi

# Ensure user's local bin is in PATH
export PATH=/root/.local/bin:/home/app/.local/bin:${PATH}

if [ ! -e /dev/ttyDXL ]; then
  # prefer existing USB/ACM nodes; adapt order to your host
  if [ -e /dev/ttyUSB0 ]; then
    ln -sf /dev/ttyUSB0 /dev/ttyDXL
  elif [ -e /dev/ttyACM0 ]; then
    ln -sf /dev/ttyACM0 /dev/ttyDXL
  fi
fi

exec "${@:-/bin/bash}"
