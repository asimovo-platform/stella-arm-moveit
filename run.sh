#!/bin/bash

echo "Navigating to the workspace."
cd /home/asimovo

echo "Sourcing the ROS 2 Humble setup."
source /opt/ros/humble/setup.bash
sudo apt update
echo "Initializing and updating rosdep."
sudo rosdep init
rosdep update

echo "Installing required ROS 2 packages."
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-ign-ros2-control
sudo apt install ros-humble-position-controllers
sudo apt install ros-humble-gripper-controllers

echo "Building the workspace."
colcon build

echo "Sourcing the workspace setup."
source install/setup.bash

echo "Rebuilding the assets for simulation."
colcon --log-base ./assets/results-1.0.0/logs build --symlink-install

echo "Now starting the simulation, check output at:"
echo "./assets/results-1.0.0/launch.log"
echo ""
echo "#> tail -f ./assets/results-1.0.0/launch.log "
echo ""
echo "You can stop the ROS simulation by calling the ./kill.sh script."

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR=./assets/results-1.0.0/logs
nohup ros2 launch ./bringup.launch > assets/results-1.0.0/launch.log 2>&1 &
