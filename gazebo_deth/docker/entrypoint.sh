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

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# So `model://WAM-V-Base/...` etc. in exhibition_water.sdf resolve to the
# meshes installed alongside the world file.
export GZ_SIM_RESOURCE_PATH="/workspace/install/asv_exhibition/share/asv_exhibition/models:${GZ_SIM_RESOURCE_PATH}"

exec ros2 launch asv_exhibition exhibition.launch.py "$@"
