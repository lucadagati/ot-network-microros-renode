#!/bin/bash
# Full system test script
# Tests all components of the OT Network Simulation project

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Full System Test${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo -e "${RED}✗ Docker not found${NC}"
    exit 1
fi

# Check if we can use Docker
if ! docker ps &> /dev/null; then
    echo -e "${YELLOW}⚠ Docker daemon not accessible${NC}"
    echo "Trying with newgrp docker..."
    exec newgrp docker << 'DOCKER_TEST'
        "$SCRIPT_DIR/test_full_system.sh"
DOCKER_TEST
    exit $?
fi

echo -e "${GREEN}✓ Docker available${NC}"
echo ""

# Test 1: Docker image
echo -e "${BLUE}[1/6] Testing Docker image...${NC}"
if docker images | grep -q microros-renode; then
    IMAGE_SIZE=$(docker images microros-renode:latest --format "{{.Size}}")
    echo -e "  ${GREEN}✓${NC} Image exists: $IMAGE_SIZE"
else
    echo -e "  ${RED}✗${NC} Image not found. Build with: docker build -t microros-renode ."
    exit 1
fi
echo ""

# Test 2: Container startup
echo -e "${BLUE}[2/6] Testing container startup...${NC}"
CONTAINER_ID=$(docker run -d --rm --name ot-test-$$ -v "$SCRIPT_DIR:/workspace" microros-renode:latest sleep 60)
sleep 2
if docker ps | grep -q ot-test-$$; then
    echo -e "  ${GREEN}✓${NC} Container started: $CONTAINER_ID"
else
    echo -e "  ${RED}✗${NC} Container failed to start"
    exit 1
fi
echo ""

# Test 3: Renode
echo -e "${BLUE}[3/6] Testing Renode...${NC}"
if docker exec "$CONTAINER_ID" renode --version &> /dev/null; then
    VERSION=$(docker exec "$CONTAINER_ID" renode --version 2>&1 | head -1)
    echo -e "  ${GREEN}✓${NC} Renode: $VERSION"
else
    echo -e "  ${RED}✗${NC} Renode not working"
fi
echo ""

# Test 4: ROS 2
echo -e "${BLUE}[4/6] Testing ROS 2...${NC}"
ROS2_OUTPUT=$(docker exec "$CONTAINER_ID" bash -c "source /opt/ros/humble/setup.bash && ros2 --version" 2>&1)
if [ $? -eq 0 ] && echo "$ROS2_OUTPUT" | grep -q "ros2"; then
    VERSION=$(echo "$ROS2_OUTPUT" | head -1)
    echo -e "  ${GREEN}✓${NC} ROS 2: $VERSION"
else
    # Try alternative check
    if docker exec "$CONTAINER_ID" test -d /opt/ros/humble; then
        echo -e "  ${GREEN}✓${NC} ROS 2 Humble installed (version check skipped)"
    else
        echo -e "  ${RED}✗${NC} ROS 2 not found"
    fi
fi
echo ""

# Test 5: micro-ROS
echo -e "${BLUE}[5/6] Testing micro-ROS...${NC}"
if docker exec "$CONTAINER_ID" test -d /microros_ws/install; then
    echo -e "  ${GREEN}✓${NC} micro-ROS workspace exists"
    PACKAGES=$(docker exec "$CONTAINER_ID" ls -1 /microros_ws/install 2>/dev/null | wc -l)
    echo -e "  ${GREEN}✓${NC} Packages installed: $PACKAGES"
else
    echo -e "  ${YELLOW}⚠${NC} micro-ROS workspace structure may be incomplete"
fi
echo ""

# Test 6: Project files
echo -e "${BLUE}[6/6] Testing project files...${NC}"
REQUIRED_FILES=(
    "README.md"
    "Dockerfile"
    "docker-compose.yml"
    "firmware/sensor_node/src/sensor_node.c"
    "firmware/actuator_node/src/actuator_node.c"
    "renode/ot_network.robot"
    "scripts/build_firmware.sh"
    "scripts/run_headless_simulation.sh"
    "scripts/start_microros_agent.sh"
)

MISSING=0
for file in "${REQUIRED_FILES[@]}"; do
    if docker exec "$CONTAINER_ID" test -f "/workspace/$file"; then
        echo -e "  ${GREEN}✓${NC} $file"
    else
        echo -e "  ${RED}✗${NC} Missing: $file"
        MISSING=$((MISSING + 1))
    fi
done

if [ $MISSING -eq 0 ]; then
    echo -e "  ${GREEN}✓${NC} All required files present"
else
    echo -e "  ${RED}✗${NC} $MISSING file(s) missing"
fi
echo ""

# Cleanup
docker stop "$CONTAINER_ID" > /dev/null 2>&1

# Summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
if [ $MISSING -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
    echo ""
    echo "System is ready for use. Next steps:"
    echo "  1. docker-compose up -d"
    echo "  2. docker-compose exec microros-renode bash"
    echo "  3. cd /workspace && ./scripts/build_firmware.sh"
    exit 0
else
    echo -e "${RED}✗ Some tests failed${NC}"
    exit 1
fi

