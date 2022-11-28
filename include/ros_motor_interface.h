#pragma once

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void init_motors();

void end_cmd_vel_subscription(rcl_node_t* node_handle);

void init_motors_messaging(rcl_node_t *node_handle, rclc_executor_t *executor);