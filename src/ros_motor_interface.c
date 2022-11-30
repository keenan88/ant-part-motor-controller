#include "ros_motor_interface.h"
#include "rcCheck.h"

#define RPS_TO_RPM_RATIO (60)

#define EXECUTOR_NUMBER_COUNTER (7)
#define GEAR_REDUCTION_RATIO (10.71)

motor_interface_t fr_motor;
motor_interface_t fl_motor;
motor_interface_t rl_motor;
motor_interface_t rr_motor;
std_msgs__msg__Float64MultiArray commanded_revs_per_s;
rcl_subscription_t cmd_vel_subscriber;

gpio_config_t io_conf = {};

// M1
clearpath_motor_pins_t fr_motor_pins =
    {
        .enable_pin = 13,
        .A_pin = 27,
        .B_pin = 26,
        .HLFB_pin = 34,
};

// M2
clearpath_motor_pins_t fl_motor_pins =
    {
        .enable_pin = 25,
        .A_pin = 33,
        .B_pin = 32,
        .HLFB_pin = 39,
};

// M3
clearpath_motor_pins_t rl_motor_pins =
    {
        .enable_pin = 21,
        .A_pin = 19,
        .B_pin = 18,
        .HLFB_pin = 23,
};

// M4
clearpath_motor_pins_t rr_motor_pins =
    {
        .enable_pin = 17,
        .A_pin = 16,
        .B_pin = 4,
        .HLFB_pin = 15,
};

/**
 * @brief sets the clearpath motor velocity by converting the cmd velocity to RPM
 *
 * @param msg cmd_vel topic
 */

// static void cmd_vel_sub_callback(const void *msg)
// {
//     geometry_msgs__msg__Twist *cmd_vel_msg = (geometry_msgs__msg__Twist *)msg;
//     double linear_velocity = cmd_vel_msg->linear.x;
//     double wheel_rpm = linear_velocity * RPS_TO_RPM_RATIO * REDUCTION_RATIO / 0.195;
//     set_motor_rpm(&motor_interface.fr_motor, wheel_rpm);
//     set_motor_rpm(&motor_interface.fl_motor, wheel_rpm);
//     set_motor_rpm(&motor_interface.rl_motor, wheel_rpm);
//     set_motor_rpm(&motor_interface.rr_motor, wheel_rpm);
// }

/**
 * @brief Initalizes the motor interface for the clearpath motors
 *
 * @param node_handle
 * @param support
 * @param allocator
 * @param executor
 */

static void subscriber_timer_callback(const void * msg)
{
    std_msgs__msg__Float64MultiArray * commanded_revs_per_s = (std_msgs__msg__Float64MultiArray*) msg;

    double fr_wheel_speed = commanded_revs_per_s->data.data[0] * RPS_TO_RPM_RATIO * GEAR_REDUCTION_RATIO;
    double fl_wheel_speed = commanded_revs_per_s->data.data[1] * RPS_TO_RPM_RATIO * GEAR_REDUCTION_RATIO;
    double rr_wheel_speed = commanded_revs_per_s->data.data[2] * RPS_TO_RPM_RATIO * GEAR_REDUCTION_RATIO;
    double rl_wheel_speed = commanded_revs_per_s->data.data[3] * RPS_TO_RPM_RATIO * GEAR_REDUCTION_RATIO;

    set_motor_rpm(&fr_motor, fr_wheel_speed);
    set_motor_rpm(&fl_motor, fl_wheel_speed);
    set_motor_rpm(&rl_motor, rr_wheel_speed);
    set_motor_rpm(&rr_motor, rl_wheel_speed);

}

void init_ros_motor_interface(rcl_node_t *node_handle, rclc_support_t *support,
                              rcl_allocator_t *allocator, rclc_executor_t *executor)
{
    init_motor_interface_base(&fr_motor, &fr_motor_pins, &io_conf, 0);
    init_motor_interface_base(&fl_motor, &fl_motor_pins, &io_conf, 1);
    init_motor_interface_base(&rl_motor, &rl_motor_pins, &io_conf, 2);
    init_motor_interface_base(&rr_motor, &rr_motor_pins, &io_conf, 3);

    commanded_revs_per_s.data.capacity = 100;
    commanded_revs_per_s.data.size = 0;
    commanded_revs_per_s.data.data = (float_t*) malloc(commanded_revs_per_s.data.capacity * sizeof(float_t));

    commanded_revs_per_s.layout.dim.capacity = 100;
    commanded_revs_per_s.layout.dim.size = 0;
    commanded_revs_per_s.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(commanded_revs_per_s.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    for(size_t i =0; i < commanded_revs_per_s.layout.dim.capacity; i++) {
        commanded_revs_per_s.layout.dim.data[i].label.capacity = 20;
        commanded_revs_per_s.layout.dim.data[i].label.size = 0;
        commanded_revs_per_s.layout.dim.data[i].label.data = (char*) malloc(commanded_revs_per_s.layout.dim.data[i].label.capacity * sizeof(char));
    }

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        node_handle,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        // ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "wheel_velocity_controller/commands"));

    RCCHECK(rclc_executor_add_subscription(
        executor,
        &cmd_vel_subscriber,
        &commanded_revs_per_s,
        // &motor_interface.cmd_vel_msg,
        subscriber_timer_callback,
        // cmd_vel_sub_callback,
        ON_NEW_DATA));
}

/**
 * @brief cleans up the ros subscriber
 */
void cleanup_motor_interface(rcl_node_t* node_handle)
{
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber,
                                  node_handle));
}
