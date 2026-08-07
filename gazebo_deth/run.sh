#!/bin/bash
set -e

echo "=== Syncing workspace from Windows and starting exhibition ==="
cp -r /mnt/c/Users/user/gazebo_deth/src ~/gazebo_deth/
cd ~/gazebo_deth

colcon build
source install/setup.bash
echo "Launching ROS 2 & Gazebo (Headless Server Mode)..."
ros2 launch asv_exhibition exhibition.launch.py headless:=true
