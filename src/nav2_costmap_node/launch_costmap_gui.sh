#!/bin/bash
# Quick launch script for costmap with GUI

echo "==================================================================="
echo "   Costmap Node with Dynamic Reconfigure GUI"
echo "==================================================================="
echo ""
echo "This will launch:"
echo "  1. Costmap node with dynamic parameter support"
echo "  2. rqt_reconfigure GUI (opens in 2 seconds)"
echo ""
echo "In the GUI, you can adjust:"
echo "  - min_obstacle_z: Ground filtering (try 0.05 to 0.5)"
echo "  - max_obstacle_z: Ceiling filtering"
echo "  - inflation_radius: Safety buffer around obstacles"
echo "  - cost_scaling_factor: Cost gradient steepness"
echo "  - publish_frequency: Costmap update rate"
echo "  - obstacle_max_range: Maximum sensor range"
echo ""
echo "Press Ctrl+C to stop"
echo "==================================================================="
echo ""

cd ~/simple_robot_ws
source install/setup.bash
ros2 launch nav2_costmap_node costmap_with_gui.launch.py
