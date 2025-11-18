#!/bin/bash
# Deploy and test script for OT Network Simulation
# Run this script to build and test the entire project

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}OT Network Simulation - Deploy & Test${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check Docker
echo -e "${BLUE}[1/6] Checking Docker...${NC}"
if command -v docker &> /dev/null; then
    echo -e "  ${GREEN}✓${NC} Docker installed: $(docker --version)"
    if docker ps &> /dev/null; then
        echo -e "  ${GREEN}✓${NC} Docker daemon is running"
        DOCKER_AVAILABLE=true
    else
        echo -e "  ${YELLOW}⚠${NC} Docker daemon not accessible (may need sudo or docker group)"
        DOCKER_AVAILABLE=false
    fi
else
    echo -e "  ${RED}✗${NC} Docker not installed"
    DOCKER_AVAILABLE=false
fi
echo ""

# Check Docker Compose
echo -e "${BLUE}[2/6] Checking Docker Compose...${NC}"
if command -v docker-compose &> /dev/null || docker compose version &> /dev/null; then
    echo -e "  ${GREEN}✓${NC} Docker Compose available"
else
    echo -e "  ${YELLOW}⚠${NC} Docker Compose not found"
fi
echo ""

# Build Docker image
echo -e "${BLUE}[3/6] Building Docker image...${NC}"
if [ "$DOCKER_AVAILABLE" = true ]; then
    echo "  Building microros-renode image..."
    if docker build -t microros-renode:latest . 2>&1 | tee /tmp/docker_build.log; then
        echo -e "  ${GREEN}✓${NC} Docker image built successfully"
        IMAGE_BUILT=true
    else
        echo -e "  ${RED}✗${NC} Docker build failed. Check /tmp/docker_build.log"
        IMAGE_BUILT=false
    fi
else
    echo -e "  ${YELLOW}⚠${NC} Skipping Docker build (Docker not available)"
    IMAGE_BUILT=false
fi
echo ""

# Test container startup
echo -e "${BLUE}[4/6] Testing container startup...${NC}"
if [ "$IMAGE_BUILT" = true ]; then
    echo "  Starting test container..."
    CONTAINER_ID=$(docker run -d --rm --name ot-network-test -v "$SCRIPT_DIR:/workspace" microros-renode:latest sleep 60)
    if [ $? -eq 0 ]; then
        echo -e "  ${GREEN}✓${NC} Container started: $CONTAINER_ID"
        
        # Test Renode installation
        echo "  Testing Renode installation..."
        if docker exec "$CONTAINER_ID" renode --version &> /dev/null; then
            echo -e "  ${GREEN}✓${NC} Renode is installed and working"
        else
            echo -e "  ${YELLOW}⚠${NC} Renode version check failed (may still work)"
        fi
        
        # Test ROS 2
        echo "  Testing ROS 2 installation..."
        if docker exec "$CONTAINER_ID" bash -c "source /opt/ros/humble/setup.bash && ros2 --help" &> /dev/null; then
            echo -e "  ${GREEN}✓${NC} ROS 2 Humble is installed"
        else
            echo -e "  ${YELLOW}⚠${NC} ROS 2 check failed"
        fi
        
        # Test micro-ROS setup
        echo "  Testing micro-ROS setup..."
        if docker exec "$CONTAINER_ID" bash -c "source /microros_ws/install/setup.bash && echo 'micro-ROS OK'" &> /dev/null; then
            echo -e "  ${GREEN}✓${NC} micro-ROS workspace is set up"
        else
            echo -e "  ${YELLOW}⚠${NC} micro-ROS setup check failed"
        fi
        
        # Cleanup
        docker stop "$CONTAINER_ID" &> /dev/null
        echo -e "  ${GREEN}✓${NC} Container test completed"
    else
        echo -e "  ${RED}✗${NC} Failed to start container"
    fi
else
    echo -e "  ${YELLOW}⚠${NC} Skipping container test (image not built)"
fi
echo ""

# Test file structure
echo -e "${BLUE}[5/6] Testing project structure...${NC}"
REQUIRED_FILES=(
    "Dockerfile"
    "docker-compose.yml"
    "README.md"
    "REFERENCES.md"
    "firmware/sensor_node/src/sensor_node.c"
    "firmware/actuator_node/src/actuator_node.c"
    "renode/ot_network.robot"
    "scripts/build_firmware.sh"
    "scripts/run_headless_simulation.sh"
)
MISSING=0
for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
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

# Test script syntax
echo -e "${BLUE}[6/6] Testing script syntax...${NC}"
SYNTAX_ERRORS=0
for script in scripts/*.sh; do
    if bash -n "$script" 2>/dev/null; then
        echo -e "  ${GREEN}✓${NC} $(basename $script)"
    else
        echo -e "  ${RED}✗${NC} $(basename $script) has syntax errors"
        SYNTAX_ERRORS=$((SYNTAX_ERRORS + 1))
    fi
done
if [ $SYNTAX_ERRORS -eq 0 ]; then
    echo -e "  ${GREEN}✓${NC} All scripts have valid syntax"
else
    echo -e "  ${RED}✗${NC} $SYNTAX_ERRORS script(s) have errors"
fi
echo ""

# Summary
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Deploy & Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
if [ "$IMAGE_BUILT" = true ]; then
    echo -e "${GREEN}✓ Docker image built successfully${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Start container: docker-compose up -d"
    echo "  2. Enter container: docker-compose exec microros-renode bash"
    echo "  3. Build firmware: ./scripts/build_firmware.sh"
    echo "  4. Run simulation: ./scripts/run_headless_simulation.sh"
else
    echo -e "${YELLOW}⚠ Docker build skipped or failed${NC}"
    echo ""
    echo "To build manually:"
    echo "  sudo docker build -t microros-renode ."
    echo "  OR"
    echo "  Add user to docker group: sudo usermod -aG docker $USER"
    echo "  Then logout and login again"
fi
echo ""

