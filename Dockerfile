# Headless reproducible build environment for park_bot.
# GUI simulation (Gazebo + RViz) is supported on Linux via X-socket mounts
# in docker-compose.yml; on macOS/Apple Silicon, use the native quickstart.

# Slim headless base (~200MB). GUI tools (Gazebo GUI, RViz) are NOT installed
# — the container targets CI parity and reproducible colcon builds. On macOS
# run the GUI sim natively; see README for the Ubuntu X-socket option.
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# NAV2 and slam_toolbox are build-linked dependencies of the package and
# must be present for colcon build to succeed. ros-gz is intentionally
# omitted from the image — it drags the full X11 and renderer stack
# (~1GB) for a container that never runs a display. Users doing GUI
# simulation run natively; see the README.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-colcon-common-extensions \
      python3-pip \
      ros-humble-nav2-bringup \
      ros-humble-nav2-amcl \
      ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir pytest

WORKDIR /root/ros2_ws

RUN mkdir -p src
COPY . src/park_bot

# Sister packages are cloned at build time to keep this Dockerfile as the
# single source of truth for what's needed to run the sim.
RUN cd src \
  && git clone --depth 1 https://github.com/nishalangovender/fws_publisher \
  && git clone --depth 1 https://github.com/nishalangovender/fws_controller \
  && git clone --depth 1 https://github.com/nishalangovender/rf2o_laser_odometry

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Run pytest as part of image build — fails the image if kinematics regress.
RUN /bin/bash -c "cd src/park_bot && python3 -m pytest test/ -v"

COPY <<'EOF' /root/entrypoint.sh
#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
exec "$@"
EOF
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]
