#ifndef FILMPROCESSOR_MOTOR_H
#define FILMPROCESSOR_MOTOR_H

extern "C" {
    void motor_init();
    void motor_reverse();
    bool motor_is_reversed();
    void motor_duty(byte level);
    int motor_count();
    int motor_rpm();
    void motor_count_clear();
}

#endif //FILMPROCESSOR_MOTOR_H