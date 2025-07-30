#!/bin/bash

# Instalador automático para el proyecto el7009_2d_exploration
# Probado en Ubuntu con ROS 2 Jazzy

set -e

sudo apt update
sudo apt install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

pip install --user -r requirements.txt

colcon build --symlink-install

source install/setup.bash

echo "Instalación completa."
