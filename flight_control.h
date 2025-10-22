#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

typedef struct {
    float w;        // scalar part of the quaternion
    float i, j, k;  // vector parts of the quaternion
} Quaternion_vector_t;

typedef unsigned char throttle_t;

typedef struct {
    throttle_t front_left, front_right, rear_left, rear_right;
} motor_throttle_states_t;

Quaternion_vector_t quaternion_conjugate(Quaternion_vector_t q);
Quaternion_vector_t quaternion_multiply(Quaternion_vector_t q1, Quaternion_vector_t q2);
Quaternion_vector_t quaternion_normalize(Quaternion_vector_t q);

void setThrottle(throttle_t throttle, motor_throttle_states_t *mts);
#endif