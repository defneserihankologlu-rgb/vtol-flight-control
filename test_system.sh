#!/bin/bash
# Test script for VTOL system

echo "=========================================="
echo "VTOL System Test Script"
echo "=========================================="

cd ~/ucus_kontrol_ws
source install/setup.bash

echo ""
echo "1. Checking if topics exist..."
echo "----------------------------------------"
ros2 topic list | grep -E "(flight_mode|odom|imu|actuator)"

echo ""
echo "2. Checking if nodes are running..."
echo "----------------------------------------"
ros2 node list

echo ""
echo "3. Checking /flight_mode topic info..."
echo "----------------------------------------"
ros2 topic info /flight_mode 2>&1 || echo "Topic not found - make sure launch file is running!"

echo ""
echo "4. Trying to echo /flight_mode (once)..."
echo "----------------------------------------"
timeout 2 ros2 topic echo /flight_mode --once 2>&1 || echo "Could not receive message - is launch file running?"

echo ""
echo "=========================================="
echo "If topics are not found:"
echo "1. Make sure launch file is running in another terminal:"
echo "   ros2 launch ucus_bringup vtol_demo.launch.py"
echo "2. Make sure you sourced the workspace:"
echo "   source ~/ucus_kontrol_ws/install/setup.bash"
echo "=========================================="

