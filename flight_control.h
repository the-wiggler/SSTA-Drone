#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdbool.h>

typedef struct {
    float w;        // scalar part of the quaternion
    float i, j, k;  // vector parts of the quaternion
} Quaternion_vector_t;

typedef unsigned char throttle_t;

typedef struct {
    throttle_t front_left, front_right, rear_left, rear_right;
} motor_throttle_states_t;

Quaternion_vector_t quaternionConjugate(Quaternion_vector_t q);
Quaternion_vector_t quaternionMultiply(Quaternion_vector_t q1, Quaternion_vector_t q2);
Quaternion_vector_t quaternionNormalize(Quaternion_vector_t q);
void writeAngleToVector(float roll, float pitch, float yaw, Quaternion_vector_t *vec);

void setThrottle(throttle_t throttle, motor_throttle_states_t *mts);
#endif