#!/bin/bash
set -e

# Start Xvfb ourselves (rather than via xvfb-run) because this script runs as
# PID 1 in the container, and xvfb-run's SIGUSR1 readiness handshake with its
# parent shell does not fire reliably for PID 1 without a real init process.
export DISPLAY=:99
Xvfb "$DISPLAY" -screen 0 1280x1024x24 -nolisten tcp &

for i in $(seq 1 20); do
    [ -e /tmp/.X99-lock ] && break
    sleep 0.5
done

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

# So `model://WAM-V-Base/...` etc. in exhibition_water.sdf resolve to the
# meshes installed alongside the world file.
export GZ_SIM_RESOURCE_PATH="/workspace/install/asv_exhibition/share/asv_exhibition/models:${GZ_SIM_RESOURCE_PATH}"

# So the vrx::Surface / vrx::SimpleHydrodynamics plugins (built from VRX
# source in the Dockerfile's vrx_plugins stage) resolve by
# filename="libSurface.so" etc. in exhibition_water.sdf.
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/vrx_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
# libSurface.so's own transitive dependency (libWaves.so) isn't found via
# GZ_SIM_SYSTEM_PLUGIN_PATH — that only controls where gz looks for the
# *requested* plugin, not where the dynamic linker resolves its own deps.
export LD_LIBRARY_PATH="/opt/vrx_plugins:${LD_LIBRARY_PATH}"

exec ros2 launch asv_exhibition exhibition.launch.py "$@"
