#include <Arduino.h>
#include <Wire.h>
#include "driver/pcnt.h"

#include "motor.h"

extern "C" {
#define MOTOR_DIR_GPIO 12

// Reverse these pins if the direction is inverted.
#define MOTOR_ENCODER_PULSE_GPIO 36
#define MOTOR_ENCODER_DIR_GPIO 39
//#define MOTOR_ENCODER_DIR_GPIO MOTOR_DIR_GPIO

// The Aslong 60 RPM motor is 168, the new 76 RPM one is 131
//#define MOTOR_GEAR_REDUCTION 168 // gear ratio of Aslong 60 RPM
#define MOTOR_GEAR_REDUCTION 131 // gear ratio of Aslong 76 RPM

#define MOTOR_ENCODER_PRECISION 32 // pulses per internal round

#define MOTOR_PWM_GPIO 14
// Define the PWM channel (0-15)
#define MOTOR_PWM_CHANNEL 0
// Set PWM frequency (e.g., 25000 Hz)
// MAX14870 recommended max is 50kHz
#define MOTOR_PWN_FREQ 50000
// Set PWM resolution (e.g., 8 bits for a 0-255 range)
// Max Frequency = 80 MHz / 2^resolution
// For 10 bits, the max frequency is 78kHz
// 8 bits should allow us to control the RPM with <1% accuracy.
#define MOTOR_PWN_RES 8

// Maximum duty adjustment per tick.
// With 70 it effectively requires 3 ticks (60ms) to ramp up to 50 RPM (~200 duty).
#define MOTOR_ADJUST_LIMIT 40
#define MOTOR_DUTY_MAX 255
// Found that the motor does not move with lower values
// Duty 0 is still allowed to stop the motor
// TODO: Not sure if this is useful in any way. May prevent adjustments from
//       working correctly in lower ranges.
#define MOTOR_DUTY_MIN 0

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
        .hctrl_mode = PCNT_MODE_KEEP, // Keep count direction if control signal is high
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

int motor_calc_rpm(int count, int dt_ms) {
    // TODO: Rewrite in int32, be careful about overflows and rounding.
    if (dt_ms == 0 || count == 0) {
        return 0;
    }
    float minutes = static_cast<float>(dt_ms) / 1000 / 60;
    float internal_count = static_cast<float>(count);
    float external_count = internal_count / MOTOR_GEAR_REDUCTION / MOTOR_ENCODER_PRECISION;
    float rpm = external_count / minutes;
    return static_cast<int>(rpm);
}

// Calculate the number of degrees the motor external axis has rotated from count.
// One rotation is 360, two rotations 720, etc.
// This may be negative.
int motor_calc_rotation_degrees(int count) {
    return 360 * count / MOTOR_GEAR_REDUCTION / MOTOR_ENCODER_PRECISION;
}

volatile int direction = 1; // 1 is forward, -1 is backward

volatile int32_t total_count = 0;
volatile uint32_t total_count_abs = 0;
volatile uint32_t total_count_fw = 0;
volatile uint32_t total_count_bw = 0;

volatile int target_duty = 0;
volatile int target_rpm = 0;
volatile int target_rotation_per_cycle = 0; // in degrees; forever if 0
volatile int target_rotation = 0; // in degrees
volatile bool target_pause = true;

volatile int last_rpm = 0;
volatile uint32_t last_rpm_millis = 0;
volatile int last_duty = 0;


void motor_dump_status() {
    Serial.printf("motor_dump_status: direction=%d last=D:%d/RPM:%d "
                  "total_count=%d/%d/%d/%d target=D:%d/RPM:%d/RPC:%d/ROT:%d rotation=%d\n",
                  direction, last_duty, last_rpm,
                  total_count, total_count_abs, total_count_fw, total_count_bw,
                  target_duty, target_rpm, target_rotation_per_cycle, target_rotation,
                  motor_calc_rotation_degrees(total_count)
    );
}

int motor_rpm() {
    return last_rpm;
}

int motor_get_target_rpm() {
    return target_rpm;
}

int motor_duty() {
    return last_duty;
}

bool motor_is_reversed() {
    return direction < 0;
}

bool motor_is_paused() {
    return target_pause;
}

void motor_set_paused(const bool pause) {
    target_pause = pause;
}

bool motor_toggle_paused() {
    target_pause = !target_pause;
    return target_pause;
}

// Absolute motor axis orientation in degrees (0-359).
// Relative to whatever starting position the motor had on startup.
unsigned int motor_position_degrees() {
    auto rot = motor_calc_rotation_degrees(total_count);
    return static_cast<uint32_t>(rot) % 360;
}

// Set a fixed PWM duty cycle.
// level: 0-255
void motor_target_duty(const byte level) {
    target_duty = level;
    target_rpm = 0;
    target_pause = level == 0;
}

// Set the target RPM value.
// For small values this will be inaccurate, because we compare against
// truncated integer RPM values. An effective RPM of 1.99 is still considered 1 RPM
// by the monitor loop.
void motor_target_rpm(const int rpm) {
    target_duty = 0;
    target_rpm = rpm;
    target_pause = rpm == 0;
}

void motor_target_rpm_paused(const int rpm) {
    target_duty = 0;
    target_rpm = rpm;
    target_pause = true;
}

void motor_target_rotation_per_cycle(int rot) {
    target_rotation_per_cycle = rot;
    target_rotation =
        motor_calc_rotation_degrees(total_count)
        + direction * target_rotation_per_cycle;
}

void motor_flush_direction() {
    digitalWrite(MOTOR_DIR_GPIO, direction > 0 ? LOW : HIGH);
}

// FIXME: no direct control by app
void motor_reverse() {
    ledcWrite(MOTOR_PWM_CHANNEL, 0);
    last_duty = 0;
    direction *= -1;
    motor_flush_direction();
}

// This will run at a frequency of 50 Hz (every 20ms) to control the motor
void motor_monitor_task(void* params) {
    const TickType_t xPeriod = 20 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    last_rpm_millis = millis();

    // Task loop
    for (;;) {
        // Wait for next cycle
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xPeriod);

        // Get and reset count
        int16_t count = 0;
        uint32_t now = millis();
        pcnt_get_counter_value(MOTOR_PCNT_UNIT, &count);
        pcnt_counter_clear(MOTOR_PCNT_UNIT);
        total_count += count; // position
        total_count_abs += abs(count); // odometer
        if (count > 0) {
            total_count_fw += count;
        } else {
            total_count_bw += abs(count);
        }

        // Calculate RPM over last tick
        int dt = now - last_rpm_millis;
        last_rpm_millis = now;
        if (dt > 0) {
            last_rpm = motor_calc_rpm(count, dt);
        }

        // If we are paused, do nothing
        if (target_pause) {
            ledcWrite(MOTOR_PWM_CHANNEL, 0);
            last_duty = 0;
            continue;
        }

        // Check if we hit our target rotation already
        if (target_rotation_per_cycle > 0) {
            int32_t total_rotation = motor_calc_rotation_degrees(total_count);
            // The direction matters here. Here it ensures we compare in the right direction.
            if ((direction * total_rotation) >= (direction * target_rotation)) {
                // Halt the motor and set last_duty to 0 for ramp up in future ticks
                // TODO: This needs better slow down handling.
                ledcWrite(MOTOR_PWM_CHANNEL, 0);
                last_duty = 0;

                // Reverse and determine next target rotation
                direction *= -1;
                target_rotation += direction * target_rotation_per_cycle;
                motor_flush_direction();

                Serial.printf("reverse direction=%d rotation=%d target_rotation=%d\n",
                              direction,
                              total_rotation,
                              target_rotation
                );

                // No further handling until next tick
                continue;
            }
        }

        // If we have a target duty instead of rpm, or they are 0
        if (target_duty > 0 || target_rpm == 0) {
            ledcWrite(MOTOR_PWM_CHANNEL, target_duty);
            last_duty = target_duty;
            continue;
        }

        // Adjust motor load to match RPM
        // Here rpm is negative if the rpm is in the wrong direction for smoother reversal.
        // In that case the duty will end up 0 until sufficiently reduced.
        int rpm = direction * last_rpm;
        if (rpm != target_rpm) {
            int diff = target_rpm - rpm; // positive if not fast enough
            int adjust = diff; // only way now to get a precision of 1

            // Limit adjustment per tick (20ms)
            // This results in full ramp up no faster than 100ms, which seems reasonable.
            if (adjust > MOTOR_ADJUST_LIMIT) {
                adjust = MOTOR_ADJUST_LIMIT;
            }
            if (adjust < -MOTOR_ADJUST_LIMIT) {
                adjust = -MOTOR_ADJUST_LIMIT;
            }

            // Ensure the new duty is within the allowed range
            int duty = last_duty + adjust;
            if (duty > MOTOR_DUTY_MAX) {
                duty = MOTOR_DUTY_MAX;
            }
            if (duty > 0 && duty < MOTOR_DUTY_MIN) {
                duty = MOTOR_DUTY_MIN;
            }

            // Set new duty level
            ledcWrite(MOTOR_PWM_CHANNEL, duty);
            last_duty = duty;
        }
    } // monitor forever loop
}


void motor_init() {
    //
    // Pulse counter
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
    // FIXME: Hangs on first interrupt, calling convention wrong?
    //pcnt_isr_register(motor_pcnt_isr_handler, nullptr, 0, nullptr);
    //pcnt_intr_enable(MOTOR_PCNT_UNIT);

    // Start the counter
    pcnt_counter_pause(MOTOR_PCNT_UNIT);
    pcnt_counter_clear(MOTOR_PCNT_UNIT);
    pcnt_counter_resume(MOTOR_PCNT_UNIT);

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

    // Direction pin
    pinMode(MOTOR_DIR_GPIO,OUTPUT);
    motor_flush_direction();

    // Start paused
    target_pause = true;

    // Start monitor task
    TaskHandle_t xHandle = nullptr;
    xTaskCreate(motor_monitor_task,
                "motor_monitor",
                2048,
                nullptr,
                tskIDLE_PRIORITY + 2,
                &xHandle);
    configASSERT(xHandle);
}
} // extern C
