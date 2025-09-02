# syntax=docker/dockerfile:1
# Base image and platform setup (Jammy 22.04). Build for arm64 with buildx.
FROM ubuntu:22.04

# Use bash for better error handling in RUN steps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Configure noninteractive installs and default timezone/locale
ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=UTC

ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND} \
    TZ=${TZ} \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Install minimal base packages and configure tzdata + locales without prompts
RUN set -euo pipefail \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        tzdata \
        locales \
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && sed -i 's/# *en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && locale-gen en_US.UTF-8 \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# System prerequisites needed for ROS setup and application build/runtime
RUN set -euo pipefail \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        sudo \
        lsb-release \
        git \
        python3 \
        python3-pip \
        build-essential \
        udev \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Application directory and source
WORKDIR /app
COPY . /app

# Ensure install script is executable
RUN chmod +x /app/xsarm_rpi4_install.sh

# Run Interbotix/ROS install script (installs ROS 2 Humble, builds workspace, sets udev rules)
# Note: this is a long step; keep earlier layers stable to leverage caching.
RUN set -euo pipefail \
    && /app/xsarm_rpi4_install.sh -d humble -p /opt/interbotix_ws -n \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /root/.cache/pip/* /tmp/* /var/tmp/*

# Create a non-root user `app` with UID 1000 and add to dialout/plugdev for USB serial access
RUN set -euo pipefail \
    && id -u app &>/dev/null || useradd -m -u 1000 -s /bin/bash app \
    && usermod -aG dialout app || true \
    && usermod -aG plugdev app || true \
    && chown -R app:app /app || true \
    && if [ -d /opt/interbotix_ws ]; then chown -R app:app /opt/interbotix_ws; fi

# Add entrypoint and make executable
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Expose FastAPI port
EXPOSE 8000

# Install uv (astral.sh) for root (for build) and for the `app` user (runtime).
# Add both users' ~/.local/bin to PATH so uv is available during build and at runtime.
ENV PATH=/root/.local/bin:/home/app/.local/bin:${PATH}

RUN set -euo pipefail \
    && curl -LsSf https://astral.sh/uv/install.sh | sh \
    && su - app -c 'bash -lc "curl -LsSf https://astral.sh/uv/install.sh | sh"' \
    && rm -rf /root/.cache/* /home/app/.cache/* || true

# Install Python application dependencies using uv (uses pyproject.toml + uv.lock)
RUN set -euo pipefail \
    && cd /app \
    && su - app -c 'bash -lc "export PATH=\$HOME/.local/bin:\$PATH; uv sync --frozen --no-dev"' \
    && rm -rf /root/.cache/pip /*/pip_cache || true

# Switch to non-root user for runtime
USER app

# Default command: run the FastAPI app with uv (uv runs uvicorn)
CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]

# Note: build for arm64 using buildx, e.g.:
# docker buildx build --platform linux/arm64 -t interbotix-control:base .
