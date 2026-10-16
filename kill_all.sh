#!/usr/bin/env bash
# Kill all Gazebo and ROS 2 processes (sim + detector stack).
# Usage: ./kill_all.sh

echo "Killing Gazebo + ROS 2 processes..."

# Gazebo
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f gazebo

# ROS 2 launch wrappers and the detector/sim nodes
pkill -9 -f "ros2 launch"
pkill -9 -f "ros2 run"
for name in \
    yolo_detector_node laser_tracker_node people_rgbd_node people_tracker_node \
    localino_node roloc_node static_transform_publisher spawner controller_manager \
    hunav_loader hunav_agent_manager hunav_gazebo \
    controller_server smoother_server planner_server behavior_server \
    bt_navigator waypoint_follower velocity_smoother lifecycle_manager \
    map_server amcl twist_mux twist_marker joystick_relay \
    robot_state_publisher rviz2 spawn_entity robot_state_publisher \
    "topic echo" "topic hz" ; do
    pkill -9 -f "$name"
done

# The ros2 daemon can get into a bad state after hard kills; restart it
ros2 daemon stop  >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1

sleep 1
echo "Done. Remaining ROS/Gazebo processes (should be empty):"
ps -eo pid,cmd | grep -E "gzserver|gzclient|_node|ros2 launch|nav2|rviz|hunav|amcl" | grep -v grep | grep -v kill_all
echo "--- (nothing above this line = all clean) ---"