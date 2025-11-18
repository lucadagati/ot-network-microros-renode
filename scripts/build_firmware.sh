#!/bin/bash
# Build script for ROS 2 nodes (Linux host version)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Building ROS 2 nodes..."

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Create firmware workspace
FIRMWARE_WS="$PROJECT_ROOT/firmware_ws"
mkdir -p "$FIRMWARE_WS/src"

# Copy nodes to workspace
cp -r "$PROJECT_ROOT/firmware/sensor_node" "$FIRMWARE_WS/src/"
cp -r "$PROJECT_ROOT/firmware/actuator_node" "$FIRMWARE_WS/src/"

cd "$FIRMWARE_WS"

# Build with colcon
echo "Building with colcon..."
colcon build --packages-select sensor_node actuator_node

echo "Build complete!"
echo "Binaries are in:"
echo "  - $FIRMWARE_WS/install/sensor_node/lib/sensor_node/sensor_node"
echo "  - $FIRMWARE_WS/install/actuator_node/lib/actuator_node/actuator_node"
echo ""
echo "To run:"
echo "  source $FIRMWARE_WS/install/setup.bash"
echo "  ros2 run sensor_node sensor_node"
echo "  ros2 run actuator_node actuator_node"
