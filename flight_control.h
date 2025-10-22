#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "pid.h" 

typedef struct {
    float w;        // scalar part of the quaternion
    float i, j, k;  // vector parts of the quaternion
} Quaternion_vector_t;

typedef struct {
    uint8_t front_left, front_right, rear_left, rear_right;
} motor_throttle_states_t;

// struct variable that holds the throttle values for each motor
extern motor_throttle_states_t throttle_states;

void setThrottle(uint8_t throttle);

#endif