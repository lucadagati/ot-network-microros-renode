# Simulating OT Nodes with micro-ROS and DDS Communication on Emulated MCUs

This project simulates a small Operational Technology (OT) network using microcontroller-based devices, similar to those found in factories, smart vehicles, or industrial machines. The system uses Renode to emulate STM32F4 Discovery MCUs and micro-ROS for lightweight ROS 2 communication over DDS.

## Project Overview

The system consists of:
- **Renode-emulated MCUs**: STM32F4 Discovery boards (ARM Cortex-M4)
- **micro-ROS nodes**: Lightweight ROS 2 nodes running on emulated MCUs
- **DDS backend**: Fast DDS for publish/subscribe messaging
- **Test scenario**: 
  - Sensor node: Publishes sensor data (temperature, pressure)
  - Actuator node: Subscribes to sensor data and responds when thresholds are crossed

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    HOST (Docker Container)              │
│                                                          │
│  ┌──────────────┐    ┌──────────────┐                  │
│  │ micro-ROS    │    │ micro-ROS    │                  │
│  │ Agent        │    │ Agent        │                  │
│  │ (Sensor)     │    │ (Actuator)   │                  │
│  └──────┬───────┘    └──────┬───────┘                  │
│         │                    │                          │
│         │ Serial/UART        │ Serial/UART              │
│         │                    │                          │
└─────────┼────────────────────┼──────────────────────────┘
          │                    │
          │                    │
┌─────────▼────────────────────▼──────────────────────────┐
│              Renode Emulator                            │
│                                                          │
│  ┌──────────────┐         ┌──────────────┐            │
│  │ Sensor Node  │         │ Actuator Node│            │
│  │ (MCU Emulato)│         │ (MCU Emulato)│            │
│  │              │         │              │            │
│  │ - Pubblica   │         │ - Sottoscrive│            │
│  │   temperatura│         │   temperatura│            │
│  │ - Pubblica   │         │ - Sottoscrive│            │
│  │   pressione  │         │   pressione  │            │
│  │              │         │ - Pubblica   │            │
│  │              │         │   stato      │            │
│  └──────────────┘         └──────────────┘            │
│                                                          │
│  STM32F4 Discovery (ARM Cortex-M4) emulati              │
└──────────────────────────────────────────────────────────┘
          │                    │
          │                    │
          └──────────┬─────────┘
                     │
          ┌──────────▼──────────┐
          │   DDS Backend        │
          │   (Fast DDS)         │
          │                      │
          │  Publish/Subscribe   │
          │  Communication        │
          └──────────────────────┘
```

## Project Structure

```
narsin/
├── Dockerfile              # Docker environment with micro-ROS and Renode
├── docker-compose.yml      # Docker Compose configuration
├── README.md              # This file
├── renode/                # Renode configuration files
│   ├── stm32f4_discovery.repl  # STM32F4 Discovery board description
│   └── ot_network.robot   # Renode script for network simulation
├── firmware/              # micro-ROS firmware code
│   ├── sensor_node/      # Sensor node implementation
│   │   ├── src/
│   │   │   └── sensor_node.c
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── actuator_node/    # Actuator node implementation
│       ├── src/
│       │   └── actuator_node.c
│       ├── CMakeLists.txt
│       └── package.xml
└── scripts/              # Helper scripts
    ├── build_firmware.sh # Build micro-ROS firmware
    └── run_headless_simulation.sh # Run Renode simulation
```

## Prerequisites

- Docker and Docker Compose
- ~8GB free disk space for Docker image
- (Optional) tmux or screen for SSH session management

## Quick Start

### Step 1: Build Docker Image

```bash
docker build -t microros-renode .
```

This will take 10-15 minutes on first build.

### Step 2: Start Container

**Option A: Using docker-compose (recommended)**

```bash
docker-compose up -d
docker-compose exec microros-renode bash
```

**Option B: Direct docker run**

```bash
docker run -it --rm -v $(pwd):/workspace --network host microros-renode:latest
```

### Step 3: Build Firmware

Inside the container:

```bash
cd /workspace
./scripts/build_firmware.sh
```

This will:
1. Create firmware workspace
2. Copy sensor and actuator nodes
3. Build using colcon (ROS 2 build system)

### Step 4: Run Simulation (Headless Mode for SSH)

**Terminal 1: Start Renode simulation**

```bash
./scripts/run_headless_simulation.sh
```

**Terminal 2: Start sensor agent**

```bash
./scripts/start_microros_agent.sh /tmp/sensor_node_uart sensor
```

**Terminal 3: Start actuator agent**

```bash
./scripts/start_microros_agent.sh /tmp/actuator_node_uart actuator
```

**Terminal 4: Monitor topics**

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /sensor/temperature
ros2 topic echo /sensor/pressure
ros2 topic echo /actuator/status
```

### Using tmux (for SSH)

```bash
# Start tmux session
tmux new -s ot-network

# Split windows (Ctrl+B, then ")
# Run commands in different panes

# Detach: Ctrl+B, then d
# Reattach: tmux attach -t ot-network
```

## System Components

### Sensor Node

**What it does:**
- Simulates an industrial sensor measuring temperature and pressure
- Publishes data every second (1 Hz) on two topics:
  - `/sensor/temperature` - Temperature in °C (base: 25°C ± variation)
  - `/sensor/pressure` - Pressure in kPa (base: 101.3 kPa ± variation)

**Behavior:**
```c
// Generates simulated values with random variation
temperature = 25.0°C + random(-2°C, +2°C)
pressure = 101.3 kPa + random(-5 kPa, +5 kPa)
```

### Actuator Node

**What it does:**
- Monitors sensor data
- Activates when critical thresholds are exceeded
- Publishes status on `/actuator/status`

**Configured thresholds:**
- **Temperature:** > 50°C → Activate actuator
- **Pressure:** > 100 kPa → Activate actuator

**Behavior:**
```c
if (temperature > 50°C || pressure > 100 kPa) {
    actuator_status = ACTIVE (1.0)
    // Simulates action: alarm, valve, cooling system, etc.
} else {
    actuator_status = INACTIVE (0.0)
}
```

## Communication Flow

### 1. Data Publication (Sensor → DDS)
```
Sensor Node (Emulated MCU)
    ↓ (micro-ROS)
Serial/UART
    ↓
micro-ROS Agent (Host)
    ↓ (DDS)
Fast DDS Backend
    ↓
Topics: /sensor/temperature, /sensor/pressure
```

### 2. Data Subscription (DDS → Actuator)
```
Topics: /sensor/temperature, /sensor/pressure
    ↓ (DDS)
Fast DDS Backend
    ↓
micro-ROS Agent (Host)
    ↓
Serial/UART
    ↓ (micro-ROS)
Actuator Node (Emulated MCU)
    ↓
Process data → Check thresholds → Publish status
```

### 3. Status Publication (Actuator → DDS)
```
Actuator Node (Emulated MCU)
    ↓ (micro-ROS)
Serial/UART
    ↓
micro-ROS Agent (Host)
    ↓ (DDS)
Fast DDS Backend
    ↓
Topic: /actuator/status
```

## Working Remotely via SSH

### Headless Mode

Renode can run in headless mode (without GUI), perfect for SSH:

```bash
renode --console --disable-xwt -e "s @renode/ot_network.robot"
```

The `--console` flag enables console mode, `--disable-xwt` disables GUI.

### Serial/UART Communication

In headless mode, Renode uses named pipes (FIFO) to simulate serial ports:

- `/tmp/sensor_node_uart`: UART for sensor node
- `/tmp/actuator_node_uart`: UART for actuator node

micro-ROS agents connect to these pipes as if they were real serial ports.

## Monitoring the System

Once running, you can monitor:

```bash
# See all topics
ros2 topic list

# Monitor temperature
ros2 topic echo /sensor/temperature

# Monitor pressure
ros2 topic echo /sensor/pressure

# Monitor actuator status
ros2 topic echo /actuator/status

# See node information
ros2 node list
ros2 node info /sensor_node
ros2 node info /actuator_node
```

## Troubleshooting

### Docker Issues

**Problem:** Permission denied
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Logout and login again, OR
newgrp docker
```

### micro-ROS Agent Issues

**Problem:** Agent not found
```bash
# Build agent
source /microros_ws/install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source /microros_ws/install/setup.bash
```

### Renode Issues

**Problem:** Renode not starting
```bash
# Check Renode installation
renode --version

# Test with simple script
renode --console -e "mach create test; quit"
```

### Firmware Build Issues

**Problem:** CMake can't find micro-ROS
```bash
# Ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Check if firmware workspace exists
ls -la firmware_ws/install/
```

### Named Pipes Issues

**Problem:** Named pipes not working
```bash
# Verify permissions
ls -l /tmp/*_uart
# Should be pipes (p) with rw-rw-rw- permissions
```

## Firmware Sources

The firmware (`sensor_node.c` and `actuator_node.c`) follows the pattern from the official micro-ROS tutorial:

- **Official Tutorial:** https://micro.ros.org/docs/tutorials/core/first_application_linux/
- **Pattern:** Initialization, publisher, subscriber, polling loop
- **Adaptation:** Adapted for ROS 2 standard (Linux host) instead of embedded micro-ROS

### What is Original

- ✅ Application logic: sensor value generation, threshold control
- ✅ Adaptation for ROS 2 standard (not embedded)
- ✅ Integration for OT network use case

### What Follows Tutorial Pattern

- ⚠️ Initialization structure (identical to tutorial)
- ⚠️ Publisher/subscriber pattern (identical to tutorial)
- ⚠️ Polling loop (identical to tutorial)
- ⚠️ Error handling with RCCHECK macro (common tutorial pattern)

## Development

### Building Firmware

The firmware is built using the ROS 2 build system (colcon). Each node has its own CMakeLists.txt and package.xml.

### Running Renode

Renode scripts define:
- MCU hardware (STM32F4 Discovery)
- Network configuration
- Firmware loading
- Serial port connections for debugging

### Testing

Monitor DDS topics using ROS 2 tools:
```bash
ros2 topic list
ros2 topic echo /sensor/temperature
ros2 topic echo /actuator/status
```

## Use Cases

This system can be extended for:

1. **Smart Factories**
   - Production line temperature monitoring
   - Hydraulic system pressure control
   - Automatic alarms

2. **Autonomous Vehicles**
   - Engine temperature sensors
   - Tire pressure control
   - Safety systems

3. **Industrial Automation**
   - Machine monitoring
   - Quality control
   - Predictive maintenance

## Advantages

### 1. Development without Hardware
- Test firmware before producing hardware
- Easier debugging (full system access)
- Reduced costs (no physical hardware)

### 2. Realistic Simulation
- Behavior identical to real hardware
- Safe testing of critical scenarios
- Reproducible tests

### 3. Standard DDS Communication
- Industrial standard protocol
- Interoperability with other ROS 2 systems
- Scalability (easy to add nodes)

### 4. Controlled Environment
- Deterministic tests
- Easy parameter modification
- Complete system monitoring

## References

### Official Documentation

- **micro-ROS Documentation:** https://micro.ros.org/
- **micro-ROS Tutorial:** https://micro.ros.org/docs/tutorials/core/first_application_linux/
- **Renode Documentation:** https://renode.io/
- **ROS 2 Documentation:** https://docs.ros.org/en/humble/
- **Fast DDS Documentation:** https://fast-dds.docs.eprosima.com/

### Repositories and Demos

- **antmicro/renode-microros-demo:** https://github.com/antmicro/renode-microros-demo
- **micro-ROS/micro-ROS-demos:** https://github.com/micro-ROS/micro-ROS-demos
- **micro-ROS/micro_ros_setup:** https://github.com/micro-ROS/micro_ros_setup

### Articles

- **Test-driven development of Zephyr + micro-ROS with Renode:** https://renode.io/news/test-driven-development-of-zephyr%2Bmicro-ros-with-renode/
- **Fully deterministic Linux + Zephyr/micro-ROS testing in Renode:** https://renode.io/news/fully-deterministic-linux-zephyr-micro-ros-testing-in-renode/

## Next Steps

1. Modify sensor thresholds in `firmware/actuator_node/src/actuator_node.c`
2. Add more sensor types
3. Implement more complex actuator logic
4. Add additional nodes to the network
5. Test with different Renode platforms

## License

This project uses open-source components. Check individual licenses in respective repositories.
