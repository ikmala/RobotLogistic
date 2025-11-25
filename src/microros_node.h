#pragma once
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>

void microRosInit();
void microRosSpinOnce();
void publishEncoderData(long enc[4]);
void getCmdVel(float &x, float &y, float &r);
