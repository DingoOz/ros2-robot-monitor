#!/bin/bash

echo "=== IMU Chart Demo ==="
echo "1. Starting dynamic IMU publisher..."
python3 test_imu_publisher.py &
IMU_PID=$!

echo "2. Waiting 3 seconds for data to accumulate..."
sleep 3

echo "3. Starting monitor (will run for 10 seconds, then show chart for 10 seconds)..."
echo "   - You'll see the /imu topic at 20.0 Hz"
echo "   - Press 'c' to toggle to chart view"
echo "   - Press 'q' to quit"

# Source ROS environment and run monitor
source install/setup.bash
timeout 30s ros2 run ros2_robot_monitor ros2_monitor

echo "4. Cleaning up..."
kill $IMU_PID 2>/dev/null
echo "Demo complete!"