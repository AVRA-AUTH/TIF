# ROS 2 Jazzy + Gazebo Harmonic (gz-sim8) — real buoyancy/hydrodynamics via
# VRX's own vrx::Surface / vrx::SimpleHydrodynamics plugins (see HANDOFF.md
# for why: those plugins are proven working in production, vs. Gazebo's core
# Buoyancy/Hydrodynamics systems which are unverified for this project).
# This intentionally diverges from setup_ubuntu.sh, which still targets
# Humble/22.04 for the native (non-Docker) path.
#
# Built from VRX's own source in this same `docker build`, so no separate
# local `vrx_jazzy` image is needed first — this stage reproduces that build
# recipe inline. It's the slow part of the first build (compiles VRX from
# source) but only reruns if this stage's own instructions change; ordinary
# `src/`-only changes below don't invalidate it.
FROM osrf/ros:jazzy-desktop AS vrx_plugins

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | tee /etc/apt/trusted.gpg.d/gazebo.asc > /dev/null && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-sdformat14 \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /root/vrx_ws

# Pinned to a specific commit on the jazzy branch (the one this project's own
# physics work was validated against — see HANDOFF.md). Only `vrx_gz` (source
# of libSurface.so/libSimpleHydrodynamics.so/libWaves.so, the plugins this
# project actually copies out below) and its one intra-repo build dependency
# `vrx_ros` are needed to compile them — the rest of the monorepo (`vrx_urdf`,
# the WAM-V mesh/URDF package, 57MB+) is only needed to *run* a full VRX
# simulation, never to build these three .so files. A partial+sparse clone
# (blob:none filter, sparse-checkout to just those two dirs) fetches only
# that, then `.git` is removed in this same layer: checking out a pinned
# commit is byte-identical regardless of .git's own internal churn, so this
# stays exactly as cacheable as a plain tarball fetch would be, just smaller.
ARG VRX_COMMIT=7609d1bd90ce7edb29d040a082f949e8b089c864
RUN git clone --filter=blob:none --no-checkout https://github.com/osrf/vrx.git src/vrx && \
    cd src/vrx && \
    git sparse-checkout init --cone && \
    git sparse-checkout set vrx_gz vrx_ros && \
    git checkout ${VRX_COMMIT} && \
    rm -rf .git

RUN bash -c "apt-get update && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --merge-install && \
    rm -rf /var/lib/apt/lists/*"

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
# gz-sim8 ABI in the vrx_plugins stage above, so no rebuild-from-source
# needed here. libWaves.so is libSurface.so's own wavefield dependency
# (confirmed via ldd on libSurface.so) — without it, gz sim fails to load
# libSurface.so at all ("libWaves.so: cannot open shared object file").
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
