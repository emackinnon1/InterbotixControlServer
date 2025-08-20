# FROM ubuntu:22.04
FROM --platform=linux/arm64 ghcr.io/sloretz/ros:humble-desktop-full

# Install essential packages (from xsarm_rpi4_install.sh)
RUN apt-get update && apt-get install -yq \
    curl \
    git \
    python3-pip \
    lsb-release \
    software-properties-common \
    gnupg \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN add-apt-repository universe && \
    apt-get update && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages (from install_essential_packages function)
RUN pip3 install \
    transforms3d \
    modern_robotics

# Install ROS 2 development tools (from install_ros2 function)
RUN apt-get update && apt-get install -yq \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-colcon-common-extensions \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update --include-eol-distros
RUN echo "ROSDEP VER"
RUN rosdep --version

# Create workspace directory
WORKDIR /app
RUN mkdir -p src

# Clone the required repositories (from install_ros2 function)
WORKDIR /app/src
RUN git clone -b humble https://github.com/Interbotix/interbotix_ros_core.git && \
    git clone -b humble https://github.com/Interbotix/interbotix_ros_manipulators.git && \
    git clone -b humble https://github.com/Interbotix/interbotix_ros_toolboxes.git && \
    git clone -b ros2 https://github.com/ros-planning/moveit_visual_tools.git

# Remove COLCON_IGNORE files (from install_ros2 function)
RUN rm -f \
      interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE \
      interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE \
      interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE \
      interbotix_ros_toolboxes/interbotix_rpi_toolbox/COLCON_IGNORE

# Initialize git submodules (from install_ros2 function)
WORKDIR /app/src/interbotix_ros_core
RUN git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox && \
    git submodule update --init interbotix_ros_xseries/interbotix_xs_driver

# Copy udev rules (from install_ros2 function)
WORKDIR /app/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
RUN cp 99-interbotix-udev.rules /etc/udev/rules.d/
# RUN udevadm control --reload-rules && sudo udevadm trigger
# Note: udevadm commands are skipped in Docker build as udev service is not running

WORKDIR /app

# Install known dependencies explicitly (conservative approach)
# These were identified from the rosdep check output
RUN apt-get update && apt-get install -y \
    ros-humble-dynamixel-sdk \
    ros-humble-xacro \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-joint-state-publisher \
    ros-humble-hardware-interface \
    ros-humble-ros2-controllers \
    ros-humble-ros-gz \
    ros-humble-tf-transformations \
    ros-humble-nav2-msgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-effort-controllers \
    ros-humble-gz-ros2-control \
    ros-humble-graph-msgs \
    ros-humble-moveit-common \
    ros-humble-moveit-core \
    ros-humble-moveit-ros-planning \
    ros-humble-rviz-visual-tools \
    && rm -rf /var/lib/apt/lists/*

    
    # Add sourcing commands to .bashrc so they're available in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /app/install/setup.bash" >> /root/.bashrc
    
# SHELL ["/bin/bash", "-c"]
RUN ls && echo "FUCK"
RUN cd /app && rosdep install --from-paths src -r -y
# Try to build the workspace (from install_ros2 function)
# RUN . /opt/ros/humble/setup.bash && colcon build || echo "Build completed with some failures, continuing..."

# ADD https://astral.sh/uv/install.sh /uv-installer.sh

# RUN sh /uv-installer.sh && rm /uv-installer.sh

# # Ensure the installed binary is on the `PATH`
# ENV PATH="/root/.local/bin/:$PATH"

ENTRYPOINT ["/bin/bash"]