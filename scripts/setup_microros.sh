#!/bin/bash
# Setup micro-ROS workspace and build system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Setting up micro-ROS workspace..."

# Source micro-ROS environment
if [ -f "/microros_ws/install/setup.bash" ]; then
    source /microros_ws/install/setup.bash
fi

# Create firmware workspace
FIRMWARE_WS="$PROJECT_ROOT/firmware_ws"
mkdir -p "$FIRMWARE_WS/src"

# Use micro-ROS build system to create firmware
cd "$FIRMWARE_WS"

# Create a colcon workspace structure
echo "Creating micro-ROS firmware workspace..."

# Note: In a real setup, you would use the micro-ROS build system
# to generate the firmware with proper toolchain and dependencies
echo "Workspace structure created at: $FIRMWARE_WS"
echo "Next steps:"
echo "1. Use micro-ROS build system to configure firmware"
echo "2. Build firmware with build_firmware.sh"

