#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include "SpeedyBee_F405_conf.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// STRUCTS
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    int16_t acc_roll, acc_pitch, acc_yaw;   // the linear acceleration parallel to each axis
    int16_t omega_roll, gyr_pitch, gyr_yaw; // the angular acceleration about each axis
    uint32_t timestamp;  // should be obtained from HAL_GetTick()
} BMP270_raw_data_t;

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

////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

Quaternion_vector_t quaternionConjugate(Quaternion_vector_t q);
Quaternion_vector_t quaternionMultiply(Quaternion_vector_t q1, Quaternion_vector_t q2);
Quaternion_vector_t quaternionNormalize(Quaternion_vector_t q);
void writeAngleToVector(float roll, float pitch, float yaw, Quaternion_vector_t *vec);

void setThrottle(throttle_t throttle, motor_throttle_states_t *mts);
void updateThrottleFromPID(motor_throttle_states_t *mts, orientation_correction_t oc);

void FC_LEDInit(void);

SPI_HandleTypeDef hspi1;
void BMP270_SPIInit(void);
HAL_StatusTypeDef BMP270_ReadSensorData(BMP270_raw_data_t *data);

////////////////////////////////////////////////////////////////////////////////////////////////////
// REGISTER DEFINITIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////
// BMP270 REGISTERS       //
////////////////////////////
// source: https://cdn.sparkfun.com/assets/9/a/2/9/6/bst-BMP270-ds000.pdf
#define BMI270_ACCEL_X_LSB    0x0C
#define BMI270_ACCEL_X_MSB    0x0D
#define BMI270_ACCEL_Y_LSB    0x0E
#define BMI270_ACCEL_Y_MSB    0x0F
#define BMI270_ACCEL_Z_LSB    0x10
#define BMI270_ACCEL_Z_MSB    0x11
#define BMI270_GYRO_X_LSB     0x12
#define BMI270_GYRO_X_MSB     0x13
#define BMI270_GYRO_Y_LSB     0x14
#define BMI270_GYRO_Y_MSB     0x15
#define BMI270_GYRO_Z_LSB     0x16
#define BMI270_GYRO_Z_MSB     0x17




////////////////////////////////////////////////////////////////////////////////////////////////////
// END
////////////////////////////////////////////////////////////////////////////////////////////////////



#endif