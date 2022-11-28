#include "motor_interface.h"
#include "ros_motor_interface.h"
#include "rotary_encoder_interface.h"

#include <rcl_interfaces/msg/log.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64_multi_array.h>

#include "rcCheck.h"


#define RADS_TO_RPM_CONVERSION_RATIO (9.5493)
#define INSIDE_WHEEL_RADIUS (0.095)
#define OUTSIDE_WHEEL_RADIUS (0.095)

#define EXECUTOR_NUMBER_COUNTER (7)
#define GEAR_REDUCTION_RATIO (10.71)







motor_interface_t fr_motor;
motor_interface_t fl_motor;
motor_interface_t rl_motor;
motor_interface_t rr_motor;
rcl_subscription_t cmd_vel_subscriber;
std_msgs__msg__Float64MultiArray cmd_vel_msg;

static void cmd_vel_callback(const void *msg)
{
    std_msgs__msg__Float64MultiArray* cmd_vel = (std_msgs__msg__Float64MultiArray *)msg;

    double fr_wheel_speed = cmd_vel->data.data[0] * RADS_TO_RPM_CONVERSION_RATIO * GEAR_REDUCTION_RATIO;
    double fl_wheel_speed = cmd_vel->data.data[1] * RADS_TO_RPM_CONVERSION_RATIO * GEAR_REDUCTION_RATIO;
    double rr_wheel_speed = cmd_vel->data.data[2] * RADS_TO_RPM_CONVERSION_RATIO * GEAR_REDUCTION_RATIO;
    double rl_wheel_speed = cmd_vel->data.data[3] * RADS_TO_RPM_CONVERSION_RATIO * GEAR_REDUCTION_RATIO;

    set_motor_rpm(&fr_motor, fr_wheel_speed);
    set_motor_rpm(&fl_motor, fl_wheel_speed);
    set_motor_rpm(&rl_motor, rr_wheel_speed);
    set_motor_rpm(&rr_motor, rl_wheel_speed);
}

void init_motors()
{
    gpio_config_t io_conf = {};

    // TODO - rename the motors to be the correct orientations, and get rid of the numbers.
    clearpath_motor_pins_t fr_motor_pins = {.enable_pin = 13, .A_pin = 27, .B_pin = 26, .HLFB_pin = 34 };     // M1
    init_motor_interface_base(&fr_motor, &fr_motor_pins, &io_conf, 0);

    clearpath_motor_pins_t fl_motor_pins = {.enable_pin = 25, .A_pin = 33, .B_pin = 32, .HLFB_pin = 39 };     // M2
    init_motor_interface_base(&fl_motor, &fl_motor_pins, &io_conf, 1);
 
    clearpath_motor_pins_t rl_motor_pins = {.enable_pin = 21, .A_pin = 19, .B_pin = 18, .HLFB_pin = 23};     // M3
    init_motor_interface_base(&rl_motor, &rl_motor_pins, &io_conf, 2); 

    clearpath_motor_pins_t rr_motor_pins = {.enable_pin = 17, .A_pin = 16, .B_pin = 4, .HLFB_pin = 15};     // M4
    init_motor_interface_base(&rr_motor, &rr_motor_pins, &io_conf, 3);
}

void init_motors_messaging(rcl_node_t *node_handle, rclc_executor_t *executor)
{
    cmd_vel_msg.data.capacity = 100;
    cmd_vel_msg.data.size = 0;
    cmd_vel_msg.data.data = (float_t *)malloc(cmd_vel_msg.data.capacity * sizeof(float_t));

    cmd_vel_msg.layout.dim.capacity = 100;
    cmd_vel_msg.layout.dim.size = 0;
    cmd_vel_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(cmd_vel_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    for (size_t i = 0; i < cmd_vel_msg.layout.dim.capacity; i++)
    {
        cmd_vel_msg.layout.dim.data[i].label.capacity = 20;
        cmd_vel_msg.layout.dim.data[i].label.size = 0;
        cmd_vel_msg.layout.dim.data[i].label.data = (char *)malloc(cmd_vel_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    RCCHECK(
        rclc_subscription_init_default(
            &cmd_vel_subscriber,
            node_handle,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "wheel_velocity_controller/commands"
        )
    );

    RCCHECK(
        rclc_executor_add_subscription(
            executor,
            &cmd_vel_subscriber,
            &cmd_vel_msg,
            cmd_vel_callback,
            ON_NEW_DATA
        )
    );
}

void end_cmd_vel_subscription(rcl_node_t* node_handle)
{
    RCCHECK(
        rcl_subscription_fini(
            &cmd_vel_subscriber,
            node_handle
        )
    );
}

