# Hydrus package managers layer: Python uv, Arduino CLI, Cargo
# Inherit from either the base or a ROS layer depending on use case
ARG PARENT_IMAGE
FROM ${PARENT_IMAGE}

# Ensure common env is present if base is used
ENV PATH="/root/.local/bin:${PATH}"

# Python and venv utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install uv (https://astral.sh/uv)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# Install Arduino CLI (package manager for Arduino cores/libs)
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install Cargo (Rust package manager) without baking toolchains/deps
RUN apt-get update && apt-get install -y --no-install-recommends cargo && rm -rf /var/lib/apt/lists/*

# Set cache locations to bind-mounted repo by default; can be overridden at runtime
ENV WORKDIR_PATH=/home/catkin_ws/src/hydrus-software-stack \
    UV_CACHE_DIR=/home/catkin_ws/src/hydrus-software-stack/.uv-cache \
    PIP_CACHE_DIR=/home/catkin_ws/src/hydrus-software-stack/.pip-cache \
    ARDUINO_DATA_DIR=/home/catkin_ws/src/hydrus-software-stack/.arduino15 \
    ARDUINO_SKETCHBOOK_DIR=/home/catkin_ws/src/hydrus-software-stack/Arduino \
    CARGO_HOME=/home/catkin_ws/src/hydrus-software-stack/.cargo \
    RUSTUP_HOME=/home/catkin_ws/src/hydrus-software-stack/.rustup \
    PATH="/root/.local/bin:${CARGO_HOME}/bin:${PATH}"

# Prepare directories and symlinks for caches
RUN mkdir -p "$ARDUINO_DATA_DIR" "$ARDUINO_SKETCHBOOK_DIR/libraries" "$CARGO_HOME/bin" "$RUSTUP_HOME" \
    && rm -rf /root/.arduino15 /root/Arduino \
    && ln -s "$ARDUINO_DATA_DIR" /root/.arduino15 \
    && ln -s "$ARDUINO_SKETCHBOOK_DIR" /root/Arduino
