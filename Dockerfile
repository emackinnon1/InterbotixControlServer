FROM ghcr.io/sloretz/ros:humble-desktop-full

COPY --from=ghcr.io/astral-sh/uv:0.8.11 /uv /uvx /bin/

WORKDIR /interbotix_ws
COPY ./interbotix_ws/src ./src
COPY ./interbotix_ws/install ./install

# Build the workspace
# RUN . /opt/ros/humble/setup.bash && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     colcon build

# Add sourcing commands to .bashrc so they're available in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /interbotix_ws/install/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]

# RUN . /opt/ros/humble/setup.bash && \
# RUN rosdep update && \ ### works inside interactive shell
#     rosdep install --from-paths src --ignore-src --rosdistro humble -r -y  && \
#     colcon build

ENTRYPOINT ["/bin/bash"]