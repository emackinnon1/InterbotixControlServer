#!/bin/bash
set -e

# Docker-specific Interbotix setup script
# This script contains only the essential parts needed for Docker installation

WORKSPACE_PATH="/opt/ros/workspace"

echo "Setting up Interbotix packages in Docker environment..."

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install transforms3d modern_robotics

# Install system dependencies
echo "Installing system dependencies..."
apt-get update
apt-get install -yq \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  build-essential \
  python3-colcon-common-extensions \
  git

# Setup rosdep
echo "Setting up rosdep..."
# Remove sources if they exist
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rm /etc/ros/rosdep/sources.list.d/20-default.list
fi

# Initialize rosdep sources
rosdep init || true
rosdep update --include-eol-distros

# Change to workspace directory
cd $WORKSPACE_PATH

# Remove COLCON_IGNORE files if they exist
echo "Removing COLCON_IGNORE files..."
cd src
rm -f \
  interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE \
  interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE \
  interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE \
  interbotix_ros_toolboxes/interbotix_rpi_toolbox/COLCON_IGNORE

# Initialize git submodules
echo "Initializing git submodules..."
cd interbotix_ros_core
git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox || true
git submodule update --init interbotix_ros_xseries/interbotix_xs_driver || true

# Setup udev rules
echo "Setting up udev rules..."
cd interbotix_ros_xseries/interbotix_xs_sdk
cp 99-interbotix-udev.rules /etc/udev/rules.d/ || true
udevadm control --reload-rules && udevadm trigger || true

# Return to workspace root
cd $WORKSPACE_PATH

# Source ROS environment
echo "Sourcing ROS environment..."
source /opt/ros/humble/setup.bash

# Install ROS dependencies
echo "Installing ROS package dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build packages
echo "Building packages with colcon..."
colcon build

echo "Interbotix setup complete!"