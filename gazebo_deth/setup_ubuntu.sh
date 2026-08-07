#!/bin/bash
set -e

echo "=== ASV Exhibition Ubuntu Setup Script ==="
echo "Installing ROS 2 Humble, Gazebo, and Dependencies..."

# 1. Ensure Locale is UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add ROS 2 Repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Update & Install ROS 2 Humble Desktop & Build Tools
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions libspdlog-dev

# 4. Install Simulation & Navigation Packages
echo "Installing Gazebo, Nav2, rosbridge, and Joy packages..."
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-rosbridge-server \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-joy \
    ros-humble-teleop-twist-joy

# 5. Add ROS 2 Sourcing to ~/.bashrc if not already present
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 sourcing to ~/.bashrc"
fi

echo ""
echo "========================================================"
echo " SUCCESS! ROS 2 Humble and Gazebo packages installed."
echo "========================================================"
echo "To complete setup:"
echo "1. Run: source ~/.bashrc"
echo "2. Run: colcon build"
echo "3. Run: source install/setup.bash"
echo "========================================================"
