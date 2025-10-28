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
    float current_error_roll;
    float current_error_pitch;
    float current_error_yaw;
    float previous_error_roll;
    float previous_error_pitch;
    float previous_error_yaw;
    float integral_roll;
    float integral_pitch;
    float integral_yaw;
    float previous_time;
} PID_rate_errors_t;

// attitude controller
angular_velocity_t calculateAttitudePID(Quaternion_vector_t setpoint, 
                                              Quaternion_vector_t current_state, 
                                              long time, 
                                              PID_errors_t *error_storage);

// rate controller (angular velocities)
orientation_correction_t calculateRatePID(angular_velocity_t desired_rate,
                                          angular_velocity_t current_rate,
                                          float time,
                                          PID_rate_errors_t *error_storage);

////////////////////////////////////////////////////////////////////////////////////////////////////


#endif