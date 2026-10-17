#!/usr/bin/env bash

# ROS setup scripts read optional environment variables that may be unset.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
WORKSPACE_SETUP="$REPO_DIR/workspace/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    printf 'ROS setup file not found: %s\n' "$ROS_SETUP" >&2
    exit 1
fi
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
    printf 'Workspace setup file not found: %s\n' "$WORKSPACE_SETUP" >&2
    exit 1
fi
if [[ ! -x "$REPO_DIR/.venv/bin/uvicorn" ]]; then
    printf 'Uvicorn executable not found: %s/.venv/bin/uvicorn\n' "$REPO_DIR" >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
exec "$REPO_DIR/.venv/bin/uvicorn" main:app --host 0.0.0.0 --port 8000