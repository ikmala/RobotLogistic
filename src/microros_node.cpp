#include "microros_node.h"

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t pub_enc;
std_msgs__msg__Int32MultiArray enc_msg;

float cmdX = 0, cmdY = 0, cmdR = 0;

void cmdVelCallback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = 
        (const geometry_msgs__msg__Twist *) msgin;

    cmdX = msg->linear.y;     // sesuai mapping omni
    cmdY = msg->linear.x;
    cmdR = msg->angular.z;
}

void microRosInit()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(1500);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "mega_robot_node", "", &support);

    // Subscriber cmd_vel
    rclc_subscription_init_default(
        &sub_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel_robot"
    );

    // Publisher encoder
    rclc_publisher_init_default(
        &pub_enc,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_feedback"
    );

    enc_msg.data.data = (int32_t *) malloc(sizeof(int32_t) * 4);
    enc_msg.data.size = 4;
    enc_msg.data.capacity = 4;

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor, &sub_cmd_vel, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);
}

void publishEncoderData(long enc[4])
{
    for(int i = 0; i < 4; i++){
        enc_msg.data.data[i] = enc[i];
    }
    rcl_publish(&pub_enc, &enc_msg, NULL);
}

void microRosSpinOnce()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}

void getCmdVel(float &x, float &y, float &r)
{
    x = cmdX;
    y = cmdY;
    r = cmdR;
}
