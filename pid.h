#ifndef PID_H
#define PID_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>

// 3 dimensional vector type
typedef vector_t {
    float x;
    float y;
    float z;
};

// PID Coefficients
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_t;

// PID error tracking values
typedef struct {
    vector_t current_error;
    vector_t previous_error;
    vector_t integral;
    float previous_time;
} PID_errors_t;

typedef struct {
    float w; // scalar part of the quaternion
    float x, y, z; // vector parts of the quaternion
} Quaternion_vector_t;

// attitude change rate pid controller 
extern PID_t attitude_pid;

// roll acceleration rate pid error values 
extern PID_errors_t attitude_errors;


vector_t calculate_pid(PID_t pid_coeffs, PID_errors_t *errors, vector_t setpoint, 
                       vector_t current_state);
void update_motors_from_pid(float roll_output, float pitch_output, float yaw_output,
                            float throttle);

void initialize_pid();
void pid_control(float desired_roll, float desired_pitch, float desired_yaw,
                 float throttle);

int16_t get_time(); // placeholder

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif