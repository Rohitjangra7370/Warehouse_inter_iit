#!/bin/bash

echo "Installing Warehouse Rover Dependencies..."

# Update system
sudo apt update

# Install Nav2
echo "Installing Nav2..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-util \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-bt-navigator

# Install SLAM Toolbox
echo "Installing SLAM Toolbox..."
sudo apt install -y ros-humble-slam-toolbox

# Install sensor drivers
echo "Installing Sensor Drivers..."
sudo apt install -y \
    ros-humble-rplidar-ros \
    ros-humble-usb-cam

# Install visualization
echo "Installing Visualization Tools..."
sudo apt install -y ros-humble-rviz2

# Install additional utilities
echo "Installing Utilities..."
sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools

# Install Python dependencies
echo "Installing Python Dependencies..."
pip3 install \
    opencv-python \
    pyzbar \
    pyserial \
    transforms3d

echo "Installation Complete!"
echo "Now building workspace..."

# Build workspace
cd ~/warehouse_rover_ws

# Clone explore if not present
if [ ! -d "src/m-explore-ros2" ]; then
    cd src
    git clone https://github.com/robo-friends/m-explore-ros2.git
    cd ..
fi

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

echo "Done! Source your workspace:"
echo "source ~/warehouse_rover_ws/install/setup.bash"
