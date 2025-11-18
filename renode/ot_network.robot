# Renode script for OT Network Simulation - STM32F4 Discovery
# Using STM32F4 Discovery boards

# Create sensor node MCU
mach create "sensor_node"
machine LoadPlatformDescription @platforms/boards/stm32f4_discovery.repl

# Create actuator node MCU
mach create "actuator_node"
machine LoadPlatformDescription @platforms/boards/stm32f4_discovery.repl

# Configure sensor node
mach set "sensor_node"
# UART configuration - use showAnalyzer for serial output
showAnalyzer sysbus.usart2

# Configure actuator node
mach set "actuator_node"
showAnalyzer sysbus.usart2

echo "OT Network Simulation Configuration Complete"
echo "STM32F4 Discovery boards configured"
echo "USART2 analyzers available for both nodes"
