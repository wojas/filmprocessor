#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include "driver/pcnt.h"

#include "logger.hpp"
#include "mqtt.hpp"

#include "config.h"

#include "motor.h"

extern "C" {

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
volatile int target_direction = 1; // 1 is forward, -1 is backward

volatile int32_t total_count = 0;
volatile uint32_t total_count_abs = 0;
volatile uint32_t total_count_fw = 0;
volatile uint32_t total_count_bw = 0;

volatile int task_not_delayed_count = 0;

volatile int target_duty = 0;
volatile int target_rpm = 0;
volatile int target_rotation_per_cycle = 0; // in degrees; forever if 0
volatile int target_progress = 0; // net degrees advanced per full cycle
volatile int target_rotation = 0; // in degrees
volatile bool target_pause = true;

volatile int last_rpm = 0;
volatile uint32_t last_rpm_millis = 0;
volatile int last_duty = 0;

static int rotation_delta_for_direction(int dir) {
    // Split progress across the two strokes in a cycle so alternating directions remain
    // possible while the net forward+reverse pair still advances by target_progress degrees.
    // Integer division gives the shared half; the remainder is applied to the forward leg.
    const int base = target_rotation_per_cycle;
    if (base <= 0) {
        return 0;
    }
    const int progress = target_progress;
    const int half = progress / 2;
    const int remainder = progress % 2;
    const int extra = dir > 0 ? (half + remainder) : (-half);
    int delta = base + extra;
    if (delta < 0) {
        delta = 0;
    }
    return delta;
}

// States in typical chronological order.
// Allowed transitions:
// - Idle -> RampUp
// - RampUp -> Running | RampDown
// - Running -> RampDown
// - RampDown -> Coast
// - Coast -> Idle | RampUp
// TODO: Do we need a Fault state we can always transition to immediately?
enum class State {Idle, RampUp, Running, RampDown, Coast};
volatile State state = State::Idle;
volatile uint32_t state_change_millis = 0; // last state change
volatile int state_change_rpm = 0; // rpm at state change
volatile int state_change_duty = 0; // duty at state change
volatile int state_ticks = 0; // number of iterations in the current state

const char* state_name(State st) {
    switch (st) {
    case State::Idle:
        return "idle";
    case State::RampUp:
        return "up";
    case State::Running:
        return "run";
    case State::RampDown:
        return "down";
    case State::Coast:
        return "coast";
    default:
        return ""; // should never happen
    }
}

void motor_dump_status() {
    LOGF("motor_dump_status: %s direction=%d last=D:%d/RPM:%d "
         "total_count=%d/%d/%d/%d target=D:%d/RPM:%d/RPC:%d/PROG:%d/ROT:%d/DIR:%d rotation=%d",
         state_name(state),
         direction, last_duty, last_rpm,
         total_count, total_count_abs, total_count_fw, total_count_bw,
         target_duty, target_rpm, target_rotation_per_cycle, target_progress, target_rotation,
         target_direction,
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
    int m = ((rot % 360) + 360) % 360; // avoid weird wrap for negative
    return static_cast<unsigned int>(m);
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
    const int dir = (target_direction != 0) ? target_direction : (direction >= 0 ? 1 : -1);
    const int32_t current = motor_calc_rotation_degrees(total_count);
    if (target_rotation_per_cycle <= 0) {
        target_rotation = current;
        return;
    }
    const int delta = rotation_delta_for_direction(dir);
    target_rotation = current + dir * delta;
}

void motor_target_progress(int progress) {
    target_progress = progress;
    if (target_rotation_per_cycle <= 0) {
        return;
    }
    int dir = target_direction;
    if (dir == 0) {
        dir = direction >= 0 ? 1 : -1;
    }
    const int32_t current = motor_calc_rotation_degrees(total_count);
    const int delta = rotation_delta_for_direction(dir);
    target_rotation = current + dir * delta;
}

void motor_flush_direction() {
    digitalWrite(MOTOR_DIR_GPIO, direction > 0 ? LOW : HIGH);
}

// Send batched metrics over MQTT
void _flush_stats(unsigned long msec, bool flushOnly = false) {
    static char buf[MQTT_PAYLOAD_MAX];
    static size_t offset = 0;
    static size_t header_size = 0;

    if ( (flushOnly && offset <= header_size) || (sizeof(buf) - offset < 120) ) {
        // Flush to MQTT (this copies the data)
        MQTT::publishAsync("letsroll/motor/csv", reinterpret_cast<const uint8_t*>(buf), offset);
        // Keep the header, but overwrite data
        offset = header_size;
    }
    if (flushOnly) {
        return;
    }

    if (offset == 0) {
        // Write CSV header. This will only happen once, as we reuse it.
        header_size = snprintf(buf + offset, sizeof(buf) - offset,
            "#ts,t_rpm,t_duty,t_rpc,t_prog,t_rot,t_dir,tot,tot_abs,tot_fw,tot_bw,paused,state,dir,rot,duty,rpm\n");
        offset += header_size;
    }

    offset += snprintf(buf + offset, sizeof(buf) - offset,
        "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%d,%d\n",
        msec,
        target_rpm,
        target_duty,
        target_rotation_per_cycle,
        target_progress,
        target_rotation,
        target_direction,
        total_count,
        total_count_abs,
        total_count_fw,
        total_count_bw,
        target_pause,
        state_name(state),
        direction,
        motor_calc_rotation_degrees(total_count),
        last_duty,
        last_rpm);
}

void _transition(const State newState) {
    const uint32_t now = millis();
    LOGF("[motor] transition: %s -> %s after %d ms",
         state_name(state),
         state_name(newState),
         now - state_change_millis);
    state = newState;
    state_change_millis = now;
    state_change_rpm = last_rpm;
    state_change_duty = last_duty;
    state_ticks = 0;
}

void _apply_duty(int duty) {
    if (duty < 0) duty = 0;
    if (duty > 255) duty = 255;
    last_duty = duty;
    ledcWrite(MOTOR_PWM_CHANNEL, duty);
}

// Remember previous duty levels by direction and rpm for use as future estimate
// to basically learn the right value for the current load within a cycle.
int _prev_duty[2], _prev_rpm[2];
void _set_previous(const int rpm, const int dir, const int duty) {
    if (duty == 0 || rpm == 0 || dir == 0) return;
    const int idx = dir > 0 ? 1 : 0;
    _prev_rpm[idx] = rpm;
    _prev_duty[idx] = duty;
}
int _get_previous(const int rpm, const int dir) {
    if (rpm == 0 || dir == 0) return 0;
    const int idx = dir > 0 ? 1 : 0;
    if (rpm != _prev_rpm[idx]) {
        return 0;
    }
    return _prev_duty[idx];
}

// This will run at a frequency of 50 Hz (every 20ms) to control the motor
void motor_monitor_task(void* params) {
    const TickType_t xPeriod = MOTOR_MONITOR_INTERVAL_MSEC / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    last_rpm_millis = millis();

    // Task loop
    for (;;) {
        // Log the previous cycle before the task delay, so that we can keep
        // using 'continue', while not introducing jitter into the actual control.
        if (state != State::Idle || last_duty != 0 || !target_pause) {
            _flush_stats(last_rpm_millis);
        } else {
            // only flush any existing data on pause
            _flush_stats(last_rpm_millis, true);
        }

        // Wait for next cycle
        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xPeriod);
        if (xWasDelayed == pdFALSE) {
            task_not_delayed_count++;
        }

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

        // Precalculate some things we need below
        const bool want_running = !target_pause && (target_rpm > 0 || target_duty > 0);
        const uint32_t state_age = now - state_change_millis;
        state_ticks++;

        // Feedforward estimate of required duty for the desired target_rpm
        int duty_ff = MOTOR_DUTY_FF_EXPR;
        // Override with previous steady state observation at this RPM
        int duty_estimate = _get_previous(target_rpm, target_direction);
        if (duty_estimate == 0) {
            duty_estimate = duty_ff;
        }
        if (target_duty > 0) {
            duty_estimate = target_duty;
        }

        switch (state) {
        case State::Idle:
            // Motor is paused and not turning
            _apply_duty(0);
            if (want_running) {
                _transition(State::RampUp);
            }
            break;

        case State::RampUp:
            // Ramp up from stationary to desired running speed using a duty estimate.
            // Any target_rpm matching is only done in Running, but we transition when
            // we get close to or over the target_rpm.
            // TODO: Set the desired direction here before duty, in case of flip.
            {
                if (target_direction != direction) {
                    direction = target_direction;
                    motor_flush_direction();
                }
                if (!want_running) {
                    _transition(State::RampDown);
                    continue;
                }
                if (state_age >= MOTOR_RAMP_UP_MAX_MSEC) {
                    _transition(State::Running);
                    continue;
                }
                // Transition to Running when we reach 90% of target_rpm
                if (target_rpm > 0 && last_rpm >= target_rpm*9/10) {
                    _transition(State::Running);
                    continue;
                }
                // Use duty_estimate as the initial target
                constexpr int n_ticks = MOTOR_RAMP_UP_MSEC / MOTOR_MONITOR_INTERVAL_MSEC;
                constexpr int adjust = 255 / n_ticks;
                int duty = last_duty + adjust;
                if (duty > duty_estimate) {
                    duty = duty_estimate;
                }
                _apply_duty(duty);
            }
            break;

        case State::Running:
            // Steady state running, until we need to RampDown
            // TODO: Switch from feedforward target to actual target
            // TODO: Restore previous duty in this direction for this target_rpm, if available
            // TODO: Limit allowed duty overshoot in first period of say ~300ms.
            // TODO: Only start updating the estimate after a certain period, once stabilized,
            //       or maybe only just before we transition to RampDown.
            {
                // Positive if in the desired direction
                int rpm = direction * last_rpm;  // should be positive now

                // Filter to avoid jitter
                // Directly using this had the disadvantage of delaying response, which
                // in itself caused large swings, so now we only use it for changes over
                // longer periods.
                static float rpm_filtered = 0;
                // α ≈ 0.39 → τ ≈ 40 ms
                // α ≈ 0.26 → τ ≈ 75 ms
                // α ≈ 0.18 → τ ≈ 100 ms
                constexpr float alpha = 0.15f; // 0.0–1.0, lower = smoother
                if (rpm_filtered == 0 || state_ticks <= 1) {
                    rpm_filtered = rpm;
                }
                rpm_filtered = alpha * rpm + (1 - alpha) * rpm_filtered;

                // Check if it was paused or equivalent
                if (!want_running) {
                    _transition(State::RampDown);
                    continue;
                }

                // Check if we hit our target rotation already
                if (target_rotation_per_cycle > 0) {
                    int32_t total_rotation = motor_calc_rotation_degrees(total_count);
                    // The direction matters here. Here it ensures we compare in the right direction.
                    if ((direction * total_rotation) >= (direction * target_rotation)) {
                        const int next_direction = -direction;
                        const int delta = rotation_delta_for_direction(next_direction);
                        target_direction = next_direction;
                        target_rotation = total_rotation + next_direction * delta;

                        LOGF("reverse next target_direction=%d rotation=%d target_rotation=%d delta=%d",
                             target_direction,
                             total_rotation,
                             target_rotation,
                             delta
                        );

                        // Remember the current duty level for the next cycle
                        if (abs(target_rpm - rpm) <= 2) {
                            _set_previous(target_rpm, direction, last_duty);
                        }

                        // Start slowing down in this cycle
                        if (last_duty > 2*MOTOR_ADJUST_LIMIT) {
                            _apply_duty(std::max(0, last_duty - MOTOR_ADJUST_LIMIT));
                        }

                        _transition(State::RampDown);
                        continue;
                    }
                }

                // In case of a fixed target_duty, just set that
                if (target_duty > 0) {
                    _apply_duty(target_duty);
                    continue;
                }

                // Adjust motor duty to match RPM
                if (rpm != target_rpm) {
                    int diff = target_rpm - rpm; // positive if not fast enough
                    int diff_filtered = target_rpm - rpm_filtered;

                    // Small smoother adjustments when we are near our target
                    if (target_rpm > 20) {
                        if (abs(diff) <= 2 && abs(diff_filtered) <= 1.5) {
                            if (state_ticks % 10 == 0) {
                                // Only once every 10 ticks (200ms)
                                diff = diff_filtered;
                            } else {
                                // Leave it as is.
                                diff = 0;;
                            }
                        } else if (abs(diff) <= 4 && abs(diff_filtered) <= 1.5) {
                            // Assume transient, ignore
                            diff = 0;
                        }
                    }

                    //int adjust = diff; // only way now to get a precision of 1
                    static int i_accum = 0;
                    i_accum = std::clamp(i_accum + diff, -MOTOR_I_MAX, MOTOR_I_MAX);
                    int adjust = MOTOR_KP * diff + MOTOR_KI * i_accum;

                    // Limit adjustment per tick (20ms)
                    if (adjust > MOTOR_ADJUST_LIMIT) {
                        adjust = MOTOR_ADJUST_LIMIT;
                    }
                    if (adjust < -MOTOR_ADJUST_LIMIT) {
                        adjust = -MOTOR_ADJUST_LIMIT;
                    }

                    // Ensure the new duty is within the allowed range
                    int max_duty = MOTOR_DUTY_MAX;
                    // First 300ms do not go past the estimate to prevent overshoot
                    if (state_age < 300) {
                        max_duty = duty_estimate;
                    }
                    int duty = last_duty + adjust;
                    if (duty > max_duty) {
                        duty = max_duty;
                    }
                    if (duty > 0 && duty < MOTOR_DUTY_MIN) {
                        duty = MOTOR_DUTY_MIN;
                    }

                    // Set new duty level
                    _apply_duty(duty);
                }
            }
            break;

        case State::RampDown:
            // Ramping down from a previous unknown duty to 0
            // FIXME: not needed, because we do not measure the actual rpm?
            {
                if (state_age >= MOTOR_RAMP_DOWN_MAX_MSEC) {
                    _transition(State::Coast);
                    continue;
                }
                // Assume the starting duty was the max of 255, and this is for
                // the worst case. If we were at a lower load, this stage will be done sooner.

                constexpr int n_ticks = MOTOR_RAMP_DOWN_MSEC / MOTOR_MONITOR_INTERVAL_MSEC;
                constexpr int adjust = 255 / n_ticks;
                int duty = last_duty - adjust;
                if (duty <= 0) {
                    duty = 0;
                }
                _apply_duty(duty);
                if (duty <= 0) {
                    _transition(State::Coast);
                }
            }
            break;

        case State::Coast:
            // Coasting phase around direction reversal, between RampDown and RampUp.
            // The motor may still be spinning in the wrong direction at the start of
            // this stage, it needs to bleed of that kinetic energy. We do not want
            // to immediately slam the gears into the other direction.
            // TODO: transition when the actual RPM is low enough, say under 5, if sooner.
            _apply_duty(0);

            // Currently we always set it before transitioning
            //if (last_rpm < 3 && target_direction != direction) {
            //    direction = target_direction;
            //    motor_flush_direction();
            //}

            // TODO: Pull EN* HIGH once wired
            if (last_rpm < 5 || state_age >= MOTOR_COAST_MSEC) {
                direction = target_direction;
                motor_flush_direction();

                if (!want_running) {
                    _transition(State::Idle);
                } else {
                    _transition(State::RampUp);
                }
            }
            break;
        }

    } // monitor forever loop
}


void motor_init() {
    //
    // Pulse counter
    //
    LOGF("Setting up motor pulse counter");

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
    LOGF("Setting up motor driver");

    // Set up the PWM channel
    ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
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
                4096,
                nullptr,
                tskIDLE_PRIORITY + 2,
                &xHandle);
    configASSERT(xHandle);
}
} // extern C
