#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64_multi_array.h>

#include "motor_interface.h"



void init_ros_motor_interface(rcl_node_t *node_handle, rclc_support_t *support,
                              rcl_allocator_t *allocator, rclc_executor_t *executor);

void cleanup_motor_interface(rcl_node_t* node_handle);
