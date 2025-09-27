#include <Arduino.h>
#include <Wire.h>
#include "driver/pcnt.h"

#include "motor.h"

extern "C" {

#define MOTOR_DIR_GPIO 12

#define MOTOR_ENCODER_PULSE_GPIO 36
#define MOTOR_ENCODER_DIR_GPIO 39
//#define MOTOR_ENCODER_DIR_GPIO MOTOR_DIR_GPIO

#define MOTOR_GEAR_REDUCTION 168 // gear ratio
#define MOTOR_ENCODER_PRECISION 32 // pulses per internal round

#define MOTOR_PWM_GPIO 14
// Define the PWM channel (0-15)
#define MOTOR_PWM_CHANNEL 0
// Set PWM frequency (e.g., 25000 Hz)
// MAX14870 recommended max is 50kHz
#define MOTOR_PWN_FREQ 50000
// Set PWM resolution (e.g., 8 bits for a 0-255 range)
#define MOTOR_PWN_RES 8

#define MOTOR_PCNT_UNIT PCNT_UNIT_0
#define MOTOR_PCNT_CHANNEL PCNT_CHANNEL_0

//typedef struct {
//    int pulse_gpio_num;          /*!< Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored */
//    int ctrl_gpio_num;           /*!< Control signal input GPIO number, a negative value will be ignored */
//    pcnt_ctrl_mode_t lctrl_mode; /*!< PCNT low control mode */
//    pcnt_ctrl_mode_t hctrl_mode; /*!< PCNT high control mode */
//    pcnt_count_mode_t pos_mode;  /*!< PCNT positive edge count mode */
//    pcnt_count_mode_t neg_mode;  /*!< PCNT negative edge count mode */
//    int16_t counter_h_lim;       /*!< Maximum counter value */
//    int16_t counter_l_lim;       /*!< Minimum counter value */
//    pcnt_unit_t unit;            /*!< PCNT unit number */
//    pcnt_channel_t channel;      /*!< the PCNT channel */
//} pcnt_config_t;
pcnt_config_t pcnt_config = {
    .pulse_gpio_num = MOTOR_ENCODER_PULSE_GPIO,
    .ctrl_gpio_num = MOTOR_ENCODER_DIR_GPIO,
    .lctrl_mode = PCNT_MODE_REVERSE, // Reverse count direction if control signal is low
    .hctrl_mode = PCNT_MODE_KEEP,    // Keep count direction if control signal is high
    .pos_mode = PCNT_COUNT_INC, // Increment counter on positive edge
    .neg_mode = PCNT_COUNT_DEC, // Decrement counter on negative edge
    .counter_h_lim = 32767,
    .counter_l_lim = -32767,
    .unit = MOTOR_PCNT_UNIT,
    .channel = MOTOR_PCNT_CHANNEL,
};

/*
void IRAM_ATTR __cdecl motor_pcnt_isr_handler(void *arg) {
    // TODO: implement
}
*/

unsigned long last_clear = 0;

void motor_count_clear() {
    pcnt_counter_clear(MOTOR_PCNT_UNIT);
    last_clear = millis();
}

int motor_count() {
    int16_t count = 0;
    pcnt_get_counter_value(MOTOR_PCNT_UNIT, &count);
    return count;
}

int motor_rpm() {
    int16_t count = 0;
    pcnt_get_counter_value(MOTOR_PCNT_UNIT, &count);

    unsigned long now = millis();
    float dt = now - last_clear;
    float minutes = dt / 1000 / 60;
    float internal_count = static_cast<float>(count);
    float external_count = internal_count / MOTOR_GEAR_REDUCTION / MOTOR_ENCODER_PRECISION;
    //float rpm = static_cast<float>(count) * 1000.0 * 60 / dt / MOTOR_GEAR_REDUCTION;
    float rpm = external_count / minutes;
    return static_cast<int>(rpm);
}

bool reverse = false;
int duty = 0;

bool motor_is_reversed() {
    return reverse;
}

// level: 0-255
void motor_duty(const byte level) {
    duty = level;
    ledcWrite(MOTOR_PWM_CHANNEL, level);
    motor_count_clear();
}

void motor_reverse() {
    motor_duty(0);
    reverse = !reverse;
    digitalWrite(MOTOR_DIR_GPIO,reverse ? LOW : HIGH);
}

void motor_init() {
    //
    // PCNT
    //
    Serial.println("Setting up motor pulse counter");

    pcnt_unit_config(&pcnt_config);

    // Remove noise
    pcnt_set_filter_value(MOTOR_PCNT_UNIT, 100); // 100 * (1/80MHz) = 1.25us
    pcnt_filter_enable(MOTOR_PCNT_UNIT);

    // Set watchpoint events
    // Call when it reaches 1000
    // FIXME: also for -1000?
    //pcnt_set_event_value(MOTOR_PCNT_UNIT, PCNT_EVT_H_LIM, 1000);
    //pcnt_event_enable(MOTOR_PCNT_UNIT, PCNT_EVT_H_LIM);
    //pcnt_isr_register(motor_pcnt_isr_handler, nullptr, 0, nullptr);
    //pcnt_intr_enable(MOTOR_PCNT_UNIT);

    // Start the counter
    pcnt_counter_pause(MOTOR_PCNT_UNIT);
    pcnt_counter_clear(MOTOR_PCNT_UNIT);
    pcnt_counter_resume(MOTOR_PCNT_UNIT);
    last_clear = millis();

    //
    // Motor driver
    //
    Serial.println("Setting up motor driver");

    // Set up the PWM channel
    ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWN_FREQ, MOTOR_PWN_RES);
    // Attach the PWM channel to the GPIO pin
    ledcAttachPin(MOTOR_PWM_GPIO, MOTOR_PWM_CHANNEL);
    // Set the initial duty cycle to 0 (motor off)
    ledcWrite(MOTOR_PWM_CHANNEL, 0);

    pinMode(MOTOR_DIR_GPIO,OUTPUT);
}

} // extern C
