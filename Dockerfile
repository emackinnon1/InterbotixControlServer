FROM osrf/ros:humble-desktop-full

COPY install/ /opt/ros/workspace/install/
COPY src/ /opt/ros/workspace/src/

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

CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]