#!/bin/bash
# Start micro-ROS agent for DDS communication
# Based on: https://micro.ros.org/docs/tutorials/core/first_application_linux/
# Reference: https://github.com/antmicro/renode-microros-demo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Check if micro-ROS agent is installed
if ! command -v micro_ros_agent &> /dev/null; then
    echo "Error: micro-ROS agent not found"
    echo "Installing micro-ROS agent..."
    
    # Install micro-ROS agent
    if [ -f "/microros_ws/install/setup.bash" ]; then
        source /microros_ws/install/setup.bash
        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh
        source /microros_ws/install/setup.bash
    else
        echo "Error: micro-ROS workspace not found. Please build micro-ROS first."
        exit 1
    fi
fi

# Check for UART device argument
UART_DEVICE=${1:-/tmp/sensor_node_uart}
NODE_NAME=${2:-sensor}

echo "Starting micro-ROS agent for $NODE_NAME node"
echo "UART device: $UART_DEVICE"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Start micro-ROS agent with serial transport
# Reference: https://micro.ros.org/docs/tutorials/core/first_application_linux/
ros2 run micro_ros_agent micro_ros_agent serial --dev "$UART_DEVICE" -v6

