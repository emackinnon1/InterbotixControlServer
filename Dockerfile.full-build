FROM arm64v8/ubuntu:22.04
# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essential packages
RUN apt-get update && apt-get install -y \
    curl \
    git \
    lsb-release \
    wget \
    gnupg2 \
    software-properties-common \
    build-essential \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
        ros-humble-desktop \
        ros-humble-moveit \
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

# Install ROS2 build tools
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install transforms3d modern_robotics

# Initialize rosdep
RUN rosdep init && rosdep update --include-eol-distros

# Set up ROS environment
ENV ROS_DISTRO=humble
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace and clone Interbotix packages
ENV INSTALL_PATH=/opt/interbotix_ws
RUN mkdir -p $INSTALL_PATH/src
WORKDIR $INSTALL_PATH/src

# Clone Interbotix repositories for ROS2 Humble
RUN git clone -b humble https://github.com/Interbotix/interbotix_ros_core.git && \
    git clone -b humble https://github.com/Interbotix/interbotix_ros_manipulators.git && \
    git clone -b humble https://github.com/Interbotix/interbotix_ros_toolboxes.git && \
    git clone -b ros2 https://github.com/ros-planning/moveit_visual_tools.git

# Remove COLCON_IGNORE files and initialize submodules
RUN rm interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE \
       interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE \
       interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE \
       interbotix_ros_toolboxes/interbotix_rpi_toolbox/COLCON_IGNORE && \
    cd interbotix_ros_core && \
    git submodule update --init interbotix_ros_xseries/dynamixel_workbench_toolbox && \
    git submodule update --init interbotix_ros_xseries/interbotix_xs_driver

# Install udev rules (for USB device access)
RUN cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk && \
    cp 99-interbotix-udev.rules /etc/udev/rules.d/

# Install dependencies and build packages
WORKDIR $INSTALL_PATH
RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="interbotix_xsarm_perception" || true && \
    colcon build --symlink-install

# Set up final environment
RUN echo "source $INSTALL_PATH/install/setup.bash" >> ~/.bashrc

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Set working directory back to workspace
WORKDIR $INSTALL_PATH

# Default command
CMD ["/bin/bash"]
