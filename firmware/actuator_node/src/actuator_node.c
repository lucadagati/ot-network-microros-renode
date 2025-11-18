/*
 * Actuator Node - Basato su Tutorial Ufficiale micro-ROS
 * 
 * FONTE REALE: https://micro.ros.org/docs/tutorials/core/first_application_linux/
 * Pattern basato su: micro-ROS First Application Tutorial
 * 
 * Questo codice segue il pattern del tutorial ufficiale micro-ROS per subscriber,
 * adattato per ROS 2 standard (Linux host) per compatibilità con build system.
 * La logica di controllo threshold è aggiunta per il caso d'uso OT network.
 */

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed: %d\n",(int)temp_rc); return 1;}}
#define TEMP_THRESH 50.0f
#define PRESS_THRESH 100.0f

rcl_subscription_t temp_sub, press_sub;
rcl_publisher_t status_pub;
std_msgs__msg__Float32 temp_msg, press_msg, status_msg;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_context_t context;
rcl_init_options_t init_options;
bool active = false;

void temp_cb(const void * m) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)m;
    printf("Actuator: temp=%.2f°C\n", msg->data);
    if (msg->data > TEMP_THRESH && !active) {
        active = true;
        status_msg.data = 1.0f;
        rcl_publish(&status_pub, &status_msg, NULL);
        printf("ACTUATOR ACTIVATED (temp)\n");
    } else if (msg->data <= TEMP_THRESH && active) {
        active = false;
        status_msg.data = 0.0f;
        rcl_publish(&status_pub, &status_msg, NULL);
    }
}

void press_cb(const void * m) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)m;
    printf("Actuator: press=%.2f kPa\n", msg->data);
    if (msg->data > PRESS_THRESH && !active) {
        active = true;
        status_msg.data = 1.0f;
        rcl_publish(&status_pub, &status_msg, NULL);
        printf("ACTUATOR ACTIVATED (press)\n");
    } else if (msg->data <= PRESS_THRESH && active) {
        active = false;
        status_msg.data = 0.0f;
        rcl_publish(&status_pub, &status_msg, NULL);
    }
}

int main(int argc, char * argv[]) {
    allocator = rcl_get_default_allocator();
    context = rcl_get_zero_initialized_context();
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init(argc, argv, &init_options, &context));
    
    const rcl_node_options_t node_ops = rcl_node_get_default_options();
    RCCHECK(rcl_node_init(&node, "actuator_node", "", &context, &node_ops));
    
    const rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
    RCCHECK(rcl_subscription_init(&temp_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/sensor/temperature", &sub_ops));
    RCCHECK(rcl_subscription_init(&press_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/sensor/pressure", &sub_ops));
    
    const rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
    RCCHECK(rcl_publisher_init(&status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/actuator/status", &pub_ops));
    
    printf("Actuator Node started\n");
    while(rcl_context_is_valid(&context)) {
        // Simple polling approach (pattern dal tutorial ufficiale)
        rcl_ret_t ret;
        ret = rcl_take(&temp_sub, &temp_msg, NULL, NULL);
        if (ret == RCL_RET_OK) {
            temp_cb(&temp_msg);
        }
        ret = rcl_take(&press_sub, &press_msg, NULL, NULL);
        if (ret == RCL_RET_OK) {
            press_cb(&press_msg);
        }
        usleep(100000); // 100ms
    }
    RCCHECK(rcl_subscription_fini(&temp_sub, &node));
    RCCHECK(rcl_subscription_fini(&press_sub, &node));
    RCCHECK(rcl_publisher_fini(&status_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_shutdown(&context));
    RCCHECK(rcl_init_options_fini(&init_options));
    return 0;
}
