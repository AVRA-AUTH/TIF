# Matches setup_ubuntu.sh: ROS 2 Humble + Gazebo (Ignition Fortress) on Ubuntu 22.04.
# Lets Ubuntu 24.04 (or any other) host run the exact same environment the
# native setup_ubuntu.sh produces, without changing that script.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-rosbridge-server \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    python3-colcon-common-extensions \
    libspdlog-dev \
    xvfb \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# No GPU in the container; force Gazebo's Ogre renderer onto software/llvmpipe GL
ENV LIBGL_ALWAYS_SOFTWARE=1

WORKDIR /workspace
COPY src ./src

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# rosbridge websocket, so the web_ui (run on the host) can connect
EXPOSE 9090

ENTRYPOINT ["/entrypoint.sh"]
CMD ["headless:=true"]
