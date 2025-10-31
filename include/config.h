#ifndef FILMPROCESSOR_CONFIG_H
#define FILMPROCESSOR_CONFIG_H

// GPIO that controls the motor H-bridge direction: LOW = forward, HIGH = reverse.
#define MOTOR_DIR_GPIO 12

// Reverse these pins if the direction is inverted.
#define MOTOR_ENCODER_PULSE_GPIO 36
#define MOTOR_ENCODER_DIR_GPIO 39
//#define MOTOR_ENCODER_DIR_GPIO MOTOR_DIR_GPIO

// The Aslong 60 RPM motor is 168, the new 76 RPM one is 131
//#define MOTOR_GEAR_REDUCTION 168 // gear ratio of Aslong 60 RPM
#define MOTOR_GEAR_REDUCTION 131
// pulses per internal round
#define MOTOR_ENCODER_PRECISION 32

// PWM output GPIO that drives the MAX14870 motor driver.
#define MOTOR_PWM_GPIO 14
// Define the PWM channel (0-15)
#define MOTOR_PWM_CHANNEL 0
// Set PWM frequency (e.g., 25000 Hz)
// MAX14870 recommended max is 50kHz
#define MOTOR_PWM_FREQ 50000
// Set PWM resolution (e.g., 8 bits for a 0-255 range)
// Max Frequency = 80 MHz / 2^resolution
// For 10 bits, the max frequency is 78kHz
// 8 bits should allow us to control the RPM with <1% accuracy.
#define MOTOR_PWM_RES 8

// Maximum duty adjustment per tick when running (after ramp up).
#define MOTOR_ADJUST_LIMIT 20
// FIXME: Pick a sensible number here, using 250 to distinguish from 255 for now
#define MOTOR_DUTY_MAX 250
// Found that the motor does not move with lower values
// Duty 0 is still allowed to stop the motor
// TODO: Not sure if this is useful in any way. May prevent adjustments from
//       working correctly in lower ranges.
#define MOTOR_DUTY_MIN 18

// Feedforward estimate of required duty for the desired target_rpm
// FIXME: calculate from linear fit on actual data
#define MOTOR_DUTY_FF_EXPR (20 + 3 * target_rpm)

// Integral clamp and PID gains for duty control.
#define MOTOR_I_MAX 30
#define MOTOR_KP 1
#define MOTOR_KI 0

// How often we monitor and adjust the motor. 50 Hz -> 20 ms
#define MOTOR_MONITOR_INTERVAL_MSEC 20

// Time to coast on direction reversal, after ramp down to near zero.
// This is easier on the motor and allows fluids to settle a bit.
#define MOTOR_COAST_MSEC 100

// Planned normal ramp down interval
#define MOTOR_RAMP_DOWN_MSEC 100
// Maximum time we will in ramp down before we force a switch to coast
#define MOTOR_RAMP_DOWN_MAX_MSEC 200

// Planned ramp up interval and its upper limit before forcing run state.
#define MOTOR_RAMP_UP_MSEC 100
#define MOTOR_RAMP_UP_MAX_MSEC 140

// Pulse-counter unit and channel used to track encoder ticks.
#define MOTOR_PCNT_UNIT PCNT_UNIT_0
#define MOTOR_PCNT_CHANNEL PCNT_CHANNEL_0

#endif // FILMPROCESSOR_CONFIG_H
