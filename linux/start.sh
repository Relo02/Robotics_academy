#!/bin/bash
# Start script for Linux - ROS2 Humble + Gazebo Harmonic

echo "=========================================="
echo "ROS2 Humble + Gazebo Harmonic - Linux"
echo "=========================================="

# Allow X11 connections from Docker
echo "[1/4] Configuring X11 permissions..."
xhost +local:docker > /dev/null 2>&1

docker exec -it ros2-gazebo bash
