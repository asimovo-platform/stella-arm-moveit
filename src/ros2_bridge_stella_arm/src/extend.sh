#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#ign topic -t /control_base_link_link1 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link1_link2 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link2_link3 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link3_link4 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link4_link5 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link4_link6 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link4_link7 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link7_link8 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link8_link12 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link8_link10 -m ignition.msgs.Double -p "data: 1.0"
#ign topic -t /control_link8_link9 -m ignition.msgs.Double -p "data: 1.0"


ros2 topic pub --once /control_base_link_link1 std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /control_link1_link2 std_msgs/msg/Float64 '{data: 1.2}'
ros2 topic pub --once /control_link2_link3 std_msgs/msg/Float64 '{data: -1.6}'
ros2 topic pub --once /control_link3_link4 std_msgs/msg/Float64 '{data: 3.14}'
#ros2 topic pub --once /control_link4_link5 std_msgs/msg/Float64 '{data: 1.0}'
#ros2 topic pub --once /control_link4_link6 std_msgs/msg/Float64 '{data: 1.0}'
ros2 topic pub --once /control_link4_link7 std_msgs/msg/Float64 '{data: -1.0}'
#ros2 topic pub --once /control_link7_link8 std_msgs/msg/Float64 '{data: 1.0}'
#ros2 topic pub --once /control_link8_link12 std_msgs/msg/Float64 '{data: 1.0}'
#ros2 topic pub --once /control_link8_link10 std_msgs/msg/Float64 '{data: 1.0}'
ros2 topic pub --once /control_link8_link9 std_msgs/msg/Float64 '{data: 1.0}'
