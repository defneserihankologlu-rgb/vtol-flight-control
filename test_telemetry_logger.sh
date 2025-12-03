#!/bin/bash
# Quick test script for telemetry logger

echo "=========================================="
echo "Telemetry Logger Test"
echo "=========================================="

cd ~/ucus_kontrol_ws
source install/setup.bash

echo ""
echo "1. Checking if rosbag2_py is available..."
python3 -c "from rosbag2_py import SequentialWriter; print('✅ rosbag2_py available')" 2>/dev/null || echo "❌ rosbag2_py not available - install with: sudo apt install ros-humble-rosbag2-py"

echo ""
echo "2. Testing telemetry logger node (will run for 5 seconds)..."
timeout 5 ros2 run ucus_bringup telemetry_logger_node.py --ros-args -p auto_start:=true 2>&1 | head -20 || echo "Node test completed"

echo ""
echo "3. Checking for log files..."
if [ -d ~/ucus_kontrol_ws/logs ]; then
    echo "   Log directory exists: ~/ucus_kontrol_ws/logs"
    ls -lh ~/ucus_kontrol_ws/logs/ | tail -5
else
    echo "   Log directory does not exist yet (will be created on first run)"
fi

echo ""
echo "=========================================="
echo "Usage:"
echo "  ros2 launch ucus_bringup vtol_with_viz.launch.py use_logging:=true"
echo "  ros2 service call /telemetry_logger/start std_srvs/srv/SetBool '{data: true}'"
echo "  ros2 service call /telemetry_logger/stop std_srvs/srv/SetBool '{data: true}'"
echo "=========================================="

