#!/bin/bash

# Start udev daemon for device management
/lib/systemd/systemd-udevd --daemon

# Reload udev rules and trigger device detection
udevadm control --reload-rules && udevadm trigger

# Source ROS environment
source /opt/ros/humble/setup.bash
source /InterbotixControlServer/install/setup.bash

# Start the FastAPI server using uv from cargo bin
uv run uvicorn main:app --host 0.0.0.0 --port 8000
