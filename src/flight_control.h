#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

////////////////////////////////////////////////////////////////////////////////////////////////////
// STRUCTS
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    int16_t acc_roll, acc_pitch, acc_yaw;   // the angular acceleration data taken from the BMI270
    int16_t gyr_roll, gyr_pitch, gyr_yaw;   // the attitude data taken from the BMI270
    uint32_t timestamp;  // should be obtained from HAL_GetTick()
} BMI270_raw_data_t;

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
void BMI270_SPIInit(void);
HAL_StatusTypeDef BMI270_ReadSensorData(BMI270_raw_data_t *data);

////////////////////////////////////////////////////////////////////////////////////////////////////
// REGISTER DEFINITIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////
// BMI270 REGISTERS       //
////////////////////////////
// source: https://cdn.sparkfun.com/assets/9/a/2/9/6/bst-bmi270-ds000.pdf

// Register Addresses (from BMI270 datasheet, Table 2)
#define BMI270_CHIP_ID              0x00
#define BMI270_STATUS               0x03  // status information

// accelerometer data
#define BMI270_ACC_X_LSB            0x0C
#define BMI270_ACC_X_MSB            0x0D
#define BMI270_ACC_Y_LSB            0x0E
#define BMI270_ACC_Y_MSB            0x0F
#define BMI270_ACC_Z_LSB            0x10
#define BMI270_ACC_Z_MSB            0x11

// gyroscope data
#define BMI270_GYR_X_LSB            0x12
#define BMI270_GYR_X_MSB            0x13
#define BMI270_GYR_Y_LSB            0x14
#define BMI270_GYR_Y_MSB            0x15
#define BMI270_GYR_Z_LSB            0x16
#define BMI270_GYR_Z_MSB            0x17

// temperatuer registers
#define BMI270_TEMP_LSB             0x20
#define BMI270_TEMP_MSB             0x21

// configuration registers
#define BMI270_ACC_CONF             0x40  // accelerometer configuration
#define BMI270_GYR_CONF             0x42  // gyroscope configuration
#define BMI270_INT_MAP_DATA         0x58  // interrupt mapping for data ready
#define BMI270_INT_IO_CONF          0x53  // interrupt pin configuration
#define BMI270_FIFO_WATERMARK       0x4E  // FIFO watermark level

// FIFO registers
#define BMI270_FIFO_LENGTH_LSB      0x24
#define BMI270_FIFO_LENGTH_MSB      0x25
#define BMI270_FIFO_DATA            0x26  // FIFO output data register

// power mode registers
#define BMI270_PWR_CONF             0x7C  // power configuration
#define BMI270_PWR_CTRL             0x7D  // power control

////////////////////////////////////////////////////////////////////////////////////////////////////
// END
////////////////////////////////////////////////////////////////////////////////////////////////////



#endif