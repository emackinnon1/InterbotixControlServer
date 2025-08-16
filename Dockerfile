FROM ghcr.io/sloretz/ros:humble-ros-base

COPY install/ /opt/ros/humble/install/
COPY src/ /opt/ros/humble/src/


RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy and run the Interbotix setup script
# COPY ros-pckg-install-script.sh /tmp/ros-pckg-install-script.sh
# RUN chmod +x /tmp/ros-pckg-install-script.sh && \
#     /tmp/ros-pckg-install-script.sh && \
#     rm /tmp/ros-pckg-install-script.sh

WORKDIR /ros_ws
RUN mkdir src

COPY . /ros_ws/src

RUN . /opt/ros/humble/setup.bash

# Build the workspace using colcon
RUN colcon build --symlink-install

# Install UV and your Python app dependencies
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates

# Download the latest installer
ADD https://astral.sh/uv/install.sh /uv-installer.sh

# Run the installer then remove it
RUN sh /uv-installer.sh && rm /uv-installer.sh

# Ensure the installed binary is on the `PATH`
ENV PATH="/root/.local/bin/:$PATH"

ADD . /app

WORKDIR /app
RUN uv sync --locked

CMD ["bash", "-c", ". /ros_ws/install/setup.bash && bash"]

# CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]