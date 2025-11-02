#ifndef FILMPROCESSOR_MOTOR_H
#define FILMPROCESSOR_MOTOR_H

#include <stdint.h>

extern "C" {
    // Setup
    void motor_init();

    // Request target
    void motor_target_duty(uint8_t level);
    void motor_target_rpm(int rpm);
    void motor_target_rotation_per_cycle(int rot);
    void motor_target_progress(int progress);
    void motor_set_paused(bool pause);
    bool motor_toggle_paused();

    // Get status
    int motor_get_target_rpm();
    int motor_count();
    int motor_rpm();
    unsigned int motor_position_degrees();
    bool motor_is_reversed();
    int motor_duty();
    void motor_dump_status();
    bool motor_is_paused();
    int motor_get_target_rotation_per_cycle();
    int motor_get_target_progress();
    int motor_pid_integral();
    int motor_pid_error();
    int32_t motor_total_count_signed();
    uint32_t motor_total_count_forward();
    uint32_t motor_total_count_backward();
    int motor_direction_sign();
    int motor_state_id();
    uint32_t motor_state_age_ms();
    uint32_t motor_last_cycle_duration_ms();
    uint32_t motor_prev_cycle_duration_ms();
    int motor_last_forward_degrees();
    int motor_last_backward_degrees();
}

#endif //FILMPROCESSOR_MOTOR_H
