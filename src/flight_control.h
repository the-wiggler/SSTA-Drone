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

typedef struct {
    float roll;     // i-axis correction
    float pitch;    // j-axis correction
    float yaw;      // k-axis correction
} orientation_correction_t;

typedef struct {
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} angular_velocity_t;

Quaternion_vector_t quaternionConjugate(Quaternion_vector_t q);
Quaternion_vector_t quaternionMultiply(Quaternion_vector_t q1, Quaternion_vector_t q2);
Quaternion_vector_t quaternionNormalize(Quaternion_vector_t q);
void writeAngleToVector(float roll, float pitch, float yaw, Quaternion_vector_t *vec);

void setThrottle(throttle_t throttle, motor_throttle_states_t *mts);
void updateThrottleFromPID(motor_throttle_states_t *mts, orientation_correction_t oc);

void FC_LEDInit(void);

SPI_HandleTypeDef hspi1;
void BMI270_SPI_Init(void);
#endif