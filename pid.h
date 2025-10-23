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

orientation_correction_t calculatePid(Quaternion_vector_t setpoint, Quaternion_vector_t current_state, long time, PID_errors_t error_storage);

////////////////////////////////////////////////////////////////////////////////////////////////////


#endif