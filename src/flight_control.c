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

void BMI270_SPIInit(void) {
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

HAL_StatusTypeDef BMI270_ReadSensorData(BMI270_raw_data_t *data) {
    HAL_StatusTypeDef status;
    uint8_t buffer[12]; // 12 long
    uint8_t addr = BMI270_ACC_X_LSB | 0x80;  // Set MSB to 1 for read operation
    
    // set CS to low to select the device
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    
    // read 12 consecutive bytes starting from accel X LSB
    status = HAL_SPI_Receive(&hspi1, buffer, 12, HAL_MAX_DELAY);
    
    // set CS to high to deselect
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    
    if (status == HAL_OK) {
        // write the accelerometer data to memory
        data->acc_roll =    (int16_t)(buffer[1]  << 8 | buffer[0]);
        data->acc_pitch =   (int16_t)(buffer[3]  << 8 | buffer[2]);
        data->acc_yaw =     (int16_t)(buffer[5]  << 8 | buffer[4]);
        data->gyr_roll =    (int16_t)(buffer[7]  << 8 | buffer[6]);
        data->gyr_pitch =   (int16_t)(buffer[9]  << 8 | buffer[8]);
        data->gyr_yaw =     (int16_t)(buffer[11] << 8 | buffer[10]);
        data->timestamp = HAL_GetTick();
    }
    
    return status;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END BOARD LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
