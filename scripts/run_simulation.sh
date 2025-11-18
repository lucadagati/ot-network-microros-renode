#!/bin/bash
# Run Renode simulation for OT network

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Starting Renode simulation..."

# Check if Renode is installed
if ! command -v renode &> /dev/null; then
    echo "Error: Renode is not installed or not in PATH"
    echo "Please install Renode or use the Docker container"
    exit 1
fi

# Check if firmware is built
SENSOR_FIRMWARE="$PROJECT_ROOT/firmware_ws/src/sensor_node/build/sensor_node"
ACTUATOR_FIRMWARE="$PROJECT_ROOT/firmware_ws/src/actuator_node/build/actuator_node"

if [ ! -f "$SENSOR_FIRMWARE" ] || [ ! -f "$ACTUATOR_FIRMWARE" ]; then
    echo "Error: Firmware not found. Please run build_firmware.sh first"
    exit 1
fi

# Run Renode with the network script
RENODE_SCRIPT="$PROJECT_ROOT/renode/ot_network.robot"

echo "Launching Renode with script: $RENODE_SCRIPT"
renode "$RENODE_SCRIPT"

