# ROS 2 Jazzy + Gazebo Harmonic (gz-sim8) — real buoyancy/hydrodynamics via
# VRX's own vrx::Surface / vrx::SimpleHydrodynamics plugins, sourced from the
# locally-built vrx_jazzy image (see HANDOFF.md for why: those plugins are
# proven working in production, vs. Gazebo's core Buoyancy/Hydrodynamics
# systems which are unverified for this project). This intentionally
# diverges from setup_ubuntu.sh, which still targets Humble/22.04 for the
# native (non-Docker) path.
FROM vrx_jazzy:latest AS vrx_plugins

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-rosbridge-server \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    python3-colcon-common-extensions \
    libspdlog-dev \
    xvfb \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# No GPU in the container; force Gazebo's Ogre renderer onto software/llvmpipe GL
ENV LIBGL_ALWAYS_SOFTWARE=1

# VRX's compiled buoyancy/hydrodynamics plugins (Apache-2.0, see
# src/asv_exhibition/models/NOTICE.md) — built against this same Jazzy/
# gz-sim8 ABI in vrx_jazzy, so no rebuild-from-source needed here.
# libWaves.so is libSurface.so's own wavefield dependency (confirmed via
# ldd on libSurface.so with vrx_jazzy's env sourced) — without it, gz sim
# fails to load libSurface.so at all ("libWaves.so: cannot open shared
# object file").
COPY --from=vrx_plugins /root/vrx_ws/install/lib/libSurface.so /root/vrx_ws/install/lib/libSimpleHydrodynamics.so /root/vrx_ws/install/lib/libWaves.so /opt/vrx_plugins/

WORKDIR /workspace
COPY src ./src

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# rosbridge websocket, so the web_ui (run on the host) can connect
EXPOSE 9090

ENTRYPOINT ["/entrypoint.sh"]
CMD ["headless:=true"]
