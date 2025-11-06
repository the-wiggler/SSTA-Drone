#include "flight_control.h"
#include <stdbool.h>
#include "SpeedyBee_F405_conf.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
// QUATERNION LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////

// quaternion conjugate (inverse for unit quaternions)
// to calculate the error between two orientations, you need to "undo" the current rotation and apply the desired rotation. 
// the conjugate represents that "undo" operation
Quaternion_vector_t quaternionConjugate(Quaternion_vector_t q) {
    Quaternion_vector_t q_conj;
    q_conj.w = q.w;
    q_conj.i = -q.i;
    q_conj.j = -q.j;
    q_conj.k = -q.k;
    return q_conj;
}


// quaternion multiplication
// result = q1 * q2
// this combines two rotations into one
// if you rotate by q1 then by q2, the combined rotation is q2×q1
Quaternion_vector_t quaternionMultiply(Quaternion_vector_t q1, Quaternion_vector_t q2) {
    Quaternion_vector_t result;
    
    result.w = q1.w * q2.w - q1.i * q2.i - q1.j * q2.j - q1.k * q2.k;
    result.i = q1.w * q2.i + q1.i * q2.w + q1.j * q2.k - q1.k * q2.j;
    result.j = q1.w * q2.j - q1.i * q2.k + q1.j * q2.w + q1.k * q2.i;
    result.k = q1.w * q2.k + q1.i * q2.j - q1.j * q2.i + q1.k * q2.w;
    
    return result;
}

// normalize quaternion
// sometimes the unit vectors get a bit off from 1 because floating point math is bad, so we have
// to fix that by normalizing them again
Quaternion_vector_t quaternionNormalize(Quaternion_vector_t q) {
    float norm = sqrtf(q.w * q.w + q.i * q.i + q.j * q.j + q.k * q.k);
    
    q.w /= norm;
    q.i /= norm;
    q.j /= norm;
    q.k /= norm;
    
    return q;
}

// this function converts euler angles attitude vectors to quaternion vectors
// the function should be used to take an euler rotation and write that rotation to a specific 
// quaternion vector array. For example, if I got a rotation angle from a gyroscope I could pass them
// into this function to convert those rotations to my quaternion vector array
void writeAngleToVector(float roll, float pitch, float yaw, Quaternion_vector_t *vec) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    vec->w = cr * cp * cy + sr * sp * sy;
    vec->i = sr * cp * cy - cr * sp * sy;
    vec->j = cr * sp * cy + sr * cp * sy;
    vec->k = cr * cp * sy - sr * sp * cy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END QUATERNION LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
// THROTTLE LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
// this function changes all values of the throttle_states variable to the same
// number, this should be used when recieving a packet from the user when they 
// set a throttle value (recieved radio frequency for example)
void setThrottle(throttle_t throttle, motor_throttle_states_t *mts) {
    mts->front_left     = throttle;
    mts->front_right    = throttle;
    mts->rear_left      = throttle;
    mts->rear_right     = throttle;
}

// THROTTLE PID CORRECTION FUNCTION
void updateThrottleFromPID(motor_throttle_states_t *mts, orientation_correction_t oc) {
    // average of all motors to preserve total thrust
    float base_throttle = (mts->front_left + mts->front_right + 
                           mts->rear_left + mts->rear_right) / 4.0f;
    
    // apply corrections according to motor mixing equation:
    float fl = base_throttle + oc.pitch + oc.roll + oc.yaw;
    float fr = base_throttle + oc.pitch - oc.roll - oc.yaw;
    float rl = base_throttle - oc.pitch + oc.roll - oc.yaw;
    float rr = base_throttle - oc.pitch - oc.roll + oc.yaw;
    
    // clamp the values to valid throttle range
    mts->front_left     = (throttle_t)fmaxf(0.0f, fminf(255.0f, fl));
    mts->front_right    = (throttle_t)fmaxf(0.0f, fminf(255.0f, fr));
    mts->rear_left      = (throttle_t)fmaxf(0.0f, fminf(255.0f, rl));
    mts->rear_right     = (throttle_t)fmaxf(0.0f, fminf(255.0f, rr));
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END THROTTLE LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// BOARD LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
// this configures the blue flight controller LED on the board
void FC_LEDInit(void) {
    // enable GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // config for the  LED pin based on the SPEEDYBEE header file info and other such things
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // initialize the GPIO pin
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    // start with the light off off
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void BMP270_SPIInit(void) {
    // enable clock for SPI1
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // config the SPI pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // the | is used to configure each pin with the below settings at the same time!
    GPIO_InitStruct.Pin = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // config the CS pin
    GPIO_InitStruct.Pin = GYRO_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GYRO_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    // config SPI1
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // ~10 MHz
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi1);
}

static uint8_t BMP270_ReadRegister(uint8_t reg_addr) {
    uint8_t tx_buffer[2] = {reg_addr | 0x80, 0x00};
    uint8_t rx_buffer[2] = {0x00, 0x00};
    
    // set CS to low to select sensor
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    
    // send register address and receive data
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, 100);
    
    // set CS to high to deselect sensor
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    
    return rx_buffer[1];
}


// main function to read all sensor data
HAL_StatusTypeDef BMP270_ReadSensorData(BMP270_raw_data_t *data) {
    if (data == NULL) {
        return HAL_ERROR;
    }
    
    // read accelerometer data
    uint8_t acc_x_lsb = BMP270_ReadRegister(BMI270_ACCEL_X_LSB);
    uint8_t acc_x_msb = BMP270_ReadRegister(BMI270_ACCEL_X_MSB);
    data->acc_roll = (int16_t)(acc_x_msb << 8 | acc_x_lsb);
    
    uint8_t acc_y_lsb = BMP270_ReadRegister(BMI270_ACCEL_Y_LSB);
    uint8_t acc_y_msb = BMP270_ReadRegister(BMI270_ACCEL_Y_MSB);
    data->acc_pitch = (int16_t)(acc_y_msb << 8 | acc_y_lsb);
    
    uint8_t acc_z_lsb = BMP270_ReadRegister(BMI270_ACCEL_Z_LSB);
    uint8_t acc_z_msb = BMP270_ReadRegister(BMI270_ACCEL_Z_MSB);
    data->acc_yaw = (int16_t)(acc_z_msb << 8 | acc_z_lsb);
    
    // read gyroscope data
    uint8_t gyr_x_lsb = BMP270_ReadRegister(BMI270_GYRO_X_LSB);
    uint8_t gyr_x_msb = BMP270_ReadRegister(BMI270_GYRO_X_MSB);
    data->omega_roll = (int16_t)(gyr_x_msb << 8 | gyr_x_lsb);
    
    uint8_t gyr_y_lsb = BMP270_ReadRegister(BMI270_GYRO_Y_LSB);
    uint8_t gyr_y_msb = BMP270_ReadRegister(BMI270_GYRO_Y_MSB);
    data->gyr_pitch = (int16_t)(gyr_y_msb << 8 | gyr_y_lsb);
    
    uint8_t gyr_z_lsb = BMP270_ReadRegister(BMI270_GYRO_Z_LSB);
    uint8_t gyr_z_msb = BMP270_ReadRegister(BMI270_GYRO_Z_MSB);
    data->gyr_yaw = (int16_t)(gyr_z_msb << 8 | gyr_z_lsb);
    
    // get current system timestamp
    data->timestamp = HAL_GetTick();
    
    return HAL_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END BOARD LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
