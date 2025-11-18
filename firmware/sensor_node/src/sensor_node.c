/*
 * Sensor Node - Basato su Tutorial Ufficiale micro-ROS
 * 
 * FONTE REALE: https://micro.ros.org/docs/tutorials/core/first_application_linux/
 * Pattern basato su: micro-ROS First Application Tutorial
 * 
 * Questo codice segue il pattern del tutorial ufficiale micro-ROS,
 * adattato per ROS 2 standard (Linux host) per compatibilità con build system.
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed line %d: %d\n",__LINE__,(int)temp_rc); return 1;}}

rcl_publisher_t temp_pub, press_pub;
std_msgs__msg__Float32 temp_msg, press_msg;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_context_t context;
rcl_init_options_t init_options;

float base_temp = 25.0f;
float base_press = 101.3f;
uint32_t seed = 12345;

uint32_t myrand() { seed = seed * 1103515245 + 12345; return (seed >> 16) & 0x7fff; }
float randf(float min, float max) { return min + ((float)myrand() / 32767.0f) * (max - min); }

int main(int argc, char * argv[]) {
    allocator = rcl_get_default_allocator();
    context = rcl_get_zero_initialized_context();
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init(argc, argv, &init_options, &context));
    
    const rcl_node_options_t node_ops = rcl_node_get_default_options();
    RCCHECK(rcl_node_init(&node, "sensor_node", "", &context, &node_ops));
    
    const rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
    RCCHECK(rcl_publisher_init(&temp_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/sensor/temperature", &pub_ops));
    RCCHECK(rcl_publisher_init(&press_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/sensor/pressure", &pub_ops));
    
    printf("Sensor Node started\n");
    while(rcl_context_is_valid(&context)) {
        temp_msg.data = base_temp + randf(-2.0f, 2.0f);
        press_msg.data = base_press + randf(-5.0f, 5.0f);
        RCCHECK(rcl_publish(&temp_pub, &temp_msg, NULL));
        RCCHECK(rcl_publish(&press_pub, &press_msg, NULL));
        printf("Sensor: temp=%.2f°C, press=%.2f kPa\n", temp_msg.data, press_msg.data);
        sleep(1);
    }
    RCCHECK(rcl_publisher_fini(&temp_pub, &node));
    RCCHECK(rcl_publisher_fini(&press_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_shutdown(&context));
    RCCHECK(rcl_init_options_fini(&init_options));
    return 0;
}
