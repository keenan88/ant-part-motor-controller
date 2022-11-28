#include "motor_interface.h"
#include "esp_log.h"

#include <driver/dac.h>
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "driver/mcpwm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rcCheck.h"


/**
 * @brief This contains the interface for a clearpath motor with the
 *          velocity controller setup, with the motor direction on the A pin
 *          and the motor velocity on the B pin.
 *          The motor encoder is the velocity output
 */
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#define MAX_PWM_BITS (8191)

#define MAX_RPM (1000.0)

#define LED_PIN 2

#define RADS_TO_RPM_CONVERSION_RATIO (9.5493)

#define INSIDE_RAD (0.0975)
#define OUTSIDE_RAD (0.045)

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY (250)            // Frequency in Hertz. Set frequency at 500 Hz

/**
 * @brief converts rads to RPM
 *
 * @param rads unit : radians/s
 * @return double
 */
static double rads_to_rpm(double rads)
{
    return rads * RADS_TO_RPM_CONVERSION_RATIO;
}

/**
 * @brief Set the up gpio pins for the object
 *
 * @param interface the object
 * @param io_conf io configuration object
 */

static void setup_gpio_config(motor_interface_t *interface, gpio_config_t *io_conf)
{
    io_conf->intr_type = GPIO_INTR_DISABLE,
    io_conf->mode = GPIO_MODE_OUTPUT,
    io_conf->pin_bit_mask = ((1ULL << interface->motor_pins.enable_pin) |
                             (1ULL << interface->motor_pins.A_pin)),
    io_conf->pull_down_en = 0,
    io_conf->pull_up_en = 0;
    gpio_config(io_conf);

    gpio_set_level(interface->motor_pins.enable_pin, 1);
    gpio_set_level(interface->motor_pins.A_pin, 1);
}

/**
 * @brief Set the up pwm timer for the object
 *
 * @param interface the motor interface object
 */

static void setup_pwm_timer(motor_interface_t *interface, uint8_t channel)
{
    interface->channel = channel;
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 500 Hz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = interface->motor_pins.B_pin,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/**
 * @brief deep copies the struct clearpath motor pins into the object
 *
 * @param interface the motor interface object
 * @param pins the clearpath motor pins that are being used
 */

static void copy_motor_pins(motor_interface_t *interface, clearpath_motor_pins_t *pins)
{
    interface->motor_pins.enable_pin = pins->enable_pin;
    interface->motor_pins.A_pin = pins->A_pin;
    interface->motor_pins.B_pin = pins->B_pin;
    interface->motor_pins.HLFB_pin = pins->HLFB_pin;
}

/**
 * @brief initalizes the clearpath motor interface
 *
 * @param interface
 * @param motor_pins the gpio pins used by the motor
 * @param io_conf the io configuration object
 */

void init_motor_interface_base(motor_interface_t *interface, clearpath_motor_pins_t *motor_pins, gpio_config_t *io_conf, uint8_t channel)
{
    copy_motor_pins(interface, motor_pins);
    setup_gpio_config(interface, io_conf);
    setup_pwm_timer(interface, channel);
}

/**
 * @brief Set the motor speed and direction
 *
 * @param interface
 * @param target_rpm the target velocity of the motor
 */

void set_motor_rpm(motor_interface_t *interface, double target_rpm)
{
    uint8_t motor_direction = (target_rpm <= 0); // 0 = CW, CCW = 1
    float duty_cycle = fabs((double)target_rpm) / ((double)MAX_RPM);
    if (interface->invert_duty_cycle)
    {
        duty_cycle = fabs(1.0 - duty_cycle);
    }
    if (duty_cycle >= 1.0) {
        duty_cycle = 1;
    }
    uint32_t casted_duty_cycle = ((int)(duty_cycle * MAX_PWM_BITS)); //(MAX_PWM_BITS - ((int)(duty_cycle * MAX_PWM_BITS))); // Needs to be inverted due to logic level shifter
    gpio_set_level(interface->motor_pins.A_pin, motor_direction);
    ledc_set_duty(LEDC_MODE, interface->channel, casted_duty_cycle);
    ledc_update_duty(LEDC_MODE, interface->channel);
    interface->called_motor_direction = !motor_direction ? CCW : CW;
}

/**
 * @brief Get the called motor direction
 *
 * @param interface
 * @return int8_t
 */

int8_t get_called_motor_direction(motor_interface_t *interface)
{
    return interface->called_motor_direction;
}

/**
 * @brief Get the motor rpm
 *
 * @param interface
 * @return double : motor speed
 */

// double get_motor_rpm(motor_interface_t *interface)
// {
//     return get_percentage(&interface->motor_encoder) * MAX_RPM;
// }
