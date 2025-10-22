#ifndef PID_H
#define PID_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>
#include "flight_control.h"

// PID Coefficients
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_t;

// PID error tracking values
typedef struct {
    Quaternion_vector_t current_error;
    Quaternion_vector_t previous_error;
    Quaternion_vector_t integral;
    float previous_time;
} PID_errors_t;

typedef struct {
    float roll;     // i-axis correction
    float pitch;    // j-axis correction
    float yaw;      // k-axis correction
} orientation_correction;

// attitude change rate pid controller 
extern PID_t attitude_pid;

// roll acceleration rate pid error values 
extern Quaternion_vector_t current_attitude;
extern PID_errors_t attitude_errors;


orientation_correction calculate_pid(PID_t pid_coeffs, PID_errors_t *errors, Quaternion_vector_t setpoint, Quaternion_vector_t current_state);
void update_motors_from_pid(float roll_output, float pitch_output,
                            float yaw_output, float throttle);

void initialize_pid();
void pid_control(float desired_roll, float desired_pitch, 
                 float desired_yaw, float throttle);

int16_t get_time(); // placeholder

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif