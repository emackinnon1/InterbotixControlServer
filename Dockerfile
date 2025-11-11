FROM ghcr.io/sloretz/ros:humble-desktop

# Install dependencies only (no source code copying)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-colcon-common-extensions \
    git \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install transforms3d modern_robotics

# Setup rosdep
RUN rosdep init || true && \
    rosdep update --include-eol-distros

# Install UV
ADD https://astral.sh/uv/install.sh /uv-installer.sh
RUN sh /uv-installer.sh && rm /uv-installer.sh
ENV PATH="/root/.local/bin/:$PATH"

# Create workspace directory
RUN mkdir -p /opt/ros/workspace

# Set working directory
WORKDIR /opt/ros/workspace

# Copy and install Python app dependencies
COPY pyproject.toml uv.lock /app/
WORKDIR /app
RUN uv sync --locked

# Default command that sources ROS and runs your app
CMD ["bash", "-c", "cd /opt/ros/workspace && source /opt/ros/humble/setup.bash && cd /app && uv run uvicorn main:app --host 0.0.0.0 --port 8000"]





# FROM ghcr.io/sloretz/ros:humble-ros-base

# COPY install/ /opt/ros/humble/install/
# COPY src/ /opt/ros/humble/src/


# RUN apt-get update && apt-get install -y --no-install-recommends \
#     python3-rosdep \
#     python3-rosinstall \
#     python3-rosinstall-generator \
#     python3-wstool \
#     build-essential \
#     python3-colcon-common-extensions \
#     python3-pip \
#     git \
#     && rm -rf /var/lib/apt/lists/*

# RUN pip3 install transforms3d modern_robotics

# RUN rosdep init || true && \
#     rosdep update --include-eol-distros

# # Copy and run the Interbotix setup script
# # COPY ros-pckg-install-script.sh /tmp/ros-pckg-install-script.sh
# # RUN chmod +x /tmp/ros-pckg-install-script.sh && \
# #     /tmp/ros-pckg-install-script.sh && \
# #     rm /tmp/ros-pckg-install-script.sh

# WORKDIR /ros_ws
# RUN mkdir src

# COPY . /ros_ws/src

# RUN . /opt/ros/humble/setup.bash

# # Install ROS dependencies
# RUN rosdep install --from-paths src --ignore-src -r -y

# # Install ROS dependencies and build
# RUN cd /opt/ros/humble && \
#     . /opt/ros/humble/setup.sh && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     colcon build

# # # Build the workspace using colcon
# # RUN colcon build --symlink-install

# # Install UV and your Python app dependencies
# RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates

# # Download the latest installer
# ADD https://astral.sh/uv/install.sh /uv-installer.sh

# # Run the installer then remove it
# RUN sh /uv-installer.sh && rm /uv-installer.sh

# # Ensure the installed binary is on the `PATH`
# ENV PATH="/root/.local/bin/:$PATH"

# ADD . /app

# WORKDIR /app
# RUN uv sync --locked

# CMD ["bash", "-c", ". /ros_ws/install/setup.bash && bash"]

# # CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
