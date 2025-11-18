#!/bin/bash
# Run Renode simulation in headless mode (for SSH)
# Based on: https://renode.io/news/test-driven-development-of-zephyr%2Bmicro-ros-with-renode/
# Reference: https://github.com/antmicro/renode-microros-demo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Starting Renode simulation in headless mode..."

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
    echo "Warning: Firmware not found. Building now..."
    "$SCRIPT_DIR/build_firmware.sh"
fi

# Create named pipes for UART communication (headless mode)
mkdir -p /tmp
rm -f /tmp/sensor_node_uart /tmp/actuator_node_uart
mkfifo /tmp/sensor_node_uart
mkfifo /tmp/actuator_node_uart

echo "Created UART pipes:"
echo "  - /tmp/sensor_node_uart"
echo "  - /tmp/actuator_node_uart"
echo ""

# Update Renode script with firmware paths
RENODE_SCRIPT="$PROJECT_ROOT/renode/ot_network.robot"
TEMP_SCRIPT="/tmp/ot_network_$(date +%s).robot"

# Create temporary script with firmware paths
cat > "$TEMP_SCRIPT" << EOF
# Temporary Renode script with firmware paths
mach create "sensor_node"
machine LoadPlatformDescription @platforms/boards/hifive1.repl

mach create "actuator_node"
machine LoadPlatformDescription @platforms/boards/hifive1.repl

mach set "sensor_node"
sysbus LoadELF @$SENSOR_FIRMWARE
cpu PC 0x80000000
connector Connect sysbus.uart0 @/tmp/sensor_node_uart

mach set "actuator_node"
sysbus LoadELF @$ACTUATOR_FIRMWARE
cpu PC 0x80000000
connector Connect sysbus.uart0 @/tmp/actuator_node_uart

echo "Starting machines..."
mach set "sensor_node"
start

mach set "actuator_node"
start

echo "OT Network Simulation Running"
echo "Connect micro-ROS agents to /tmp/sensor_node_uart and /tmp/actuator_node_uart"
EOF

echo "Launching Renode in headless mode..."
echo "Script: $TEMP_SCRIPT"
echo ""
echo "In separate terminals, run:"
echo "  ./scripts/start_microros_agent.sh /tmp/sensor_node_uart sensor"
echo "  ./scripts/start_microros_agent.sh /tmp/actuator_node_uart actuator"
echo ""

# Run Renode in console mode (headless)
renode --console --disable-xwt -e "s @$TEMP_SCRIPT"

