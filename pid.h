#ifndef PID_H
#define PID_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>

// note: ive just made all of the PID types int16_t type since im not sure what processing power
// we'll have yet.

// PID Coefficients
typedef struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;
} PID_t;

// PID error tracking values
typedef struct {
    int16_t current_error;
    int16_t previous_error;
    int16_t integral;
    // previous_time variable is to track the time taken since the last PID calculation was
    // so that the d and i parts can work as they should. This isnt necessary if we use
    // an interrupt that has an exact time (then we can use a constant)
    int16_t previous_time;
} PID_errors_t;

// roll acceleration rate pid controller 
extern PID_t roll_rate_pid;
extern PID_t pitch_rate_pid;
extern PID_t yaw_rate_pid;

// roll acceleration rate pid error values 
extern PID_errors_t roll_rate_errors;
extern PID_errors_t pitch_rate_errors;
extern PID_errors_t yaw_rate_errors;

// max allowed rate
extern float max_roll_rate;
extern float max_pitch_rate;
extern float max_yaw_rate;

float calculate_pid(const PID_t *pid_coeffs, PID_errors_t *errors, int16_t setpoint,
                    int16_t current_state);
void update_motors_from_pid(int16_t roll_output, int16_t pitch_output, int16_t yaw_output,
                            int16_t throttle);
void initialize_pid();
void pid_control(int16_t desired_roll, int16_t desired_pitch, int16_t desired_yaw,
                 int16_t throttle);

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif