#!/bin/bash
# Example script demonstrating runtime parameter changes for robot_self_filter
# This script shows how to dynamically adjust filter parameters while the node is running

echo "=== Robot Self Filter - Runtime Parameter Example ==="
echo ""
echo "Make sure the self_filter node is running before executing these commands:"
echo "  ros2 launch robot_self_filter self_filter.launch.py ..."
echo ""

# Function to show usage
show_usage() {
    echo "Usage examples:"
    echo ""
    echo "1. Adjust scale for a specific link (increases filter size by 20%):"
    echo "   ros2 param set /self_filter self_see_links.BOOM.scale 1.2"
    echo ""
    echo "2. Adjust padding for a link (adds 5cm padding):"
    echo "   ros2 param set /self_filter self_see_links.BOOM.padding 0.05"
    echo ""
    echo "3. Adjust box dimensions (separate x, y, z scaling):"
    echo "   ros2 param set /self_filter self_see_links.STICK.box_scale '[1.1, 1.1, 1.7]'"
    echo "   ros2 param set /self_filter self_see_links.STICK.box_padding '[0.05, 0.22, 0.05]'"
    echo ""
    echo "4. Adjust cylinder dimensions (radial and vertical):"
    echo "   ros2 param set /self_filter self_see_links.LF_WHEEL.cylinder_scale '[1.0, 1.0]'"
    echo "   ros2 param set /self_filter self_see_links.LF_WHEEL.cylinder_padding '[0.05, 0.3]'"
    echo ""
    echo "5. Adjust minimum sensor distance:"
    echo "   ros2 param set /self_filter min_sensor_dist 2.5"
    echo ""
    echo "6. List all parameters:"
    echo "   ros2 param list /self_filter"
    echo ""
    echo "7. Get current value of a parameter:"
    echo "   ros2 param get /self_filter self_see_links.BOOM.scale"
    echo ""
}

# Check if ros2 command is available
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found. Make sure ROS 2 is sourced."
    exit 1
fi

# Check if node name argument is provided
NODE_NAME=${1:-/self_filter}

# Check if the node is running
echo "Checking if node $NODE_NAME is running..."
if ! ros2 node list | grep -q "$NODE_NAME"; then
    echo "WARNING: Node $NODE_NAME is not running."
    echo ""
    show_usage
    exit 1
fi

echo "Node $NODE_NAME is running!"
echo ""

# Interactive mode
echo "=== Interactive Parameter Adjustment ==="
echo ""
echo "Select an action:"
echo "  1) List all parameters"
echo "  2) Show link-specific parameters"
echo "  3) Adjust scale for a link"
echo "  4) Adjust padding for a link"
echo "  5) Show usage examples"
echo "  6) Exit"
echo ""

read -p "Enter choice [1-6]: " choice

case $choice in
    1)
        echo ""
        echo "All parameters for $NODE_NAME:"
        ros2 param list "$NODE_NAME"
        ;;
    2)
        echo ""
        echo "Link-specific parameters:"
        ros2 param list "$NODE_NAME" | grep "self_see_links\."
        ;;
    3)
        echo ""
        read -p "Enter link name (e.g., BOOM): " link_name
        read -p "Enter scale value (e.g., 1.2): " scale_value
        echo "Setting scale for $link_name to $scale_value..."
        ros2 param set "$NODE_NAME" "self_see_links.$link_name.scale" "$scale_value"
        ;;
    4)
        echo ""
        read -p "Enter link name (e.g., BOOM): " link_name
        read -p "Enter padding value (e.g., 0.05): " padding_value
        echo "Setting padding for $link_name to $padding_value..."
        ros2 param set "$NODE_NAME" "self_see_links.$link_name.padding" "$padding_value"
        ;;
    5)
        show_usage
        ;;
    6)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice"
        show_usage
        exit 1
        ;;
esac

echo ""
echo "Done!"
