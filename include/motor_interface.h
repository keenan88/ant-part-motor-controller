#pragma once

#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "driver/gpio.h"

#define MAX_VALUE 255
#define MAX_VEL 3.0

#define CCW (1)
#define CW (-1)
typedef enum
{
    MOTOR_CTRL_MODE_FIXED = 0,
    MOTOR_CTRL_MODE_TRIANGLE,
    MOTOR_CTRL_MODE_RECTANGLE
} expect_mode_t;

typedef struct clearpath_motor_interface_pins
{
    uint8_t enable_pin;
    uint8_t A_pin;
    uint8_t B_pin;
    uint8_t HLFB_pin;
} clearpath_motor_pins_t;

typedef struct motor_interfce
{
    rcl_subscription_t cmd_vel_subscriber;
    geometry_msgs__msg__Twist vel_msg; // not needed other than for compilation
    clearpath_motor_pins_t motor_pins;
    int8_t called_motor_direction;
    bool invert_duty_cycle;
    uint8_t channel;
} motor_interface_t;

void init_motor_interface_base(motor_interface_t *interface, clearpath_motor_pins_t *motor_pins,
                               gpio_config_t *io_conf, uint8_t channel);

uint8_t set_velocity(motor_interface_t *interface, double velocity);

void set_motor_rpm(motor_interface_t *interface, double target_rpm);

void interpret_message(motor_interface_t *interface, const void *args);

int8_t get_called_motor_direction(motor_interface_t *interface);

double get_motor_rpm(motor_interface_t *interface);