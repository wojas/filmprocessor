#ifndef FILMPROCESSOR_MOTOR_H
#define FILMPROCESSOR_MOTOR_H

extern "C" {
    // Setup
    void motor_init();

    // Direct control (deprecated)
    void motor_reverse();

    // Request target
    void motor_target_duty(byte level);
    void motor_target_rpm(int rpm);
    void motor_target_rotation_per_cycle(int rot);

    // Get status
    int motor_get_target_rpm();
    int motor_count();
    int motor_rpm();
    unsigned int motor_position_degrees();
    bool motor_is_reversed();
    int motor_duty();
    void motor_dump_status();
}

#endif //FILMPROCESSOR_MOTOR_H