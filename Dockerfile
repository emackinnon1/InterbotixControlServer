FROM ghcr.io/sloretz/ros:humble-desktop-full

COPY --from=ghcr.io/astral-sh/uv:0.8.11 /uv /uvx /bin/

# COPY /opt/ros/humble /opt/ros/humble 

WORKDIR /app
RUN mkdir src
COPY ./src/interbotix_ros_core/ ./src/interbotix_ros_core/
COPY ./src/interbotix_ros_manipulators/ ./src/interbotix_ros_manipulators/
COPY ./src/interbotix_ros_toolboxes/ ./src/interbotix_ros_toolboxes/
COPY ./src/moveit_visual_tools/ ./src/moveit_visual_tools/
COPY ./install ./install


# Build the workspace
# RUN . /opt/ros/humble/setup.bash && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     colcon build

# Add sourcing commands to .bashrc so they're available in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /app/install/setup.bash" >> /root/.bashrc

# SHELL ["/bin/bash", "-c"]

# RUN . /opt/ros/humble/setup.bash && \
# RUN rosdep update && \ ### works inside interactive shell
#     rosdep install --from-paths src --ignore-src --rosdistro humble -r -y  && \
#     colcon build

ENTRYPOINT ["/bin/bash"]