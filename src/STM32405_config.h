/*
 * SpeedyBee F405 V3 Flight Controller Pinout
 * Thanks to the Betaflight source code for providing this info <3
 */

#ifndef SPEEDYBEEF405V3_PINOUT_H
#define SPEEDYBEEF405V3_PINOUT_H

#include "stm32f4xx_hal.h"

/* ========== GPIO Pin Definitions ========== */

/* Motors (PWM/DShot outputs) */
#define MOTOR_1_PIN         GPIO_PIN_6
#define MOTOR_1_PORT        GPIOB
#define MOTOR_2_PIN         GPIO_PIN_7
#define MOTOR_2_PORT        GPIOB
#define MOTOR_3_PIN         GPIO_PIN_8
#define MOTOR_3_PORT        GPIOB
#define MOTOR_4_PIN         GPIO_PIN_9
#define MOTOR_4_PORT        GPIOB
#define MOTOR_5_PIN         GPIO_PIN_0
#define MOTOR_5_PORT        GPIOB
#define MOTOR_6_PIN         GPIO_PIN_1
#define MOTOR_6_PORT        GPIOB
#define MOTOR_7_PIN         GPIO_PIN_5
#define MOTOR_7_PORT        GPIOB
#define MOTOR_8_PIN         GPIO_PIN_4
#define MOTOR_8_PORT        GPIOB

/* Servo */
#define SERVO_1_PIN         GPIO_PIN_8
#define SERVO_1_PORT        GPIOA

/* LEDs */
#define LED_PIN             GPIO_PIN_8
#define LED_PORT            GPIOC
#define LED_STRIP_PIN       GPIO_PIN_9
#define LED_STRIP_PORT      GPIOC

/* Beeper */
#define BEEPER_PIN          GPIO_PIN_5
#define BEEPER_PORT         GPIOC

/* UART1 */
#define UART1_TX_PIN        GPIO_PIN_9
#define UART1_TX_PORT       GPIOA
#define UART1_RX_PIN        GPIO_PIN_10
#define UART1_RX_PORT       GPIOA

/* UART2 */
#define UART2_TX_PIN        GPIO_PIN_2
#define UART2_TX_PORT       GPIOA
#define UART2_RX_PIN        GPIO_PIN_3
#define UART2_RX_PORT       GPIOA

/* UART3 */
#define UART3_TX_PIN        GPIO_PIN_10
#define UART3_TX_PORT       GPIOC
#define UART3_RX_PIN        GPIO_PIN_11
#define UART3_RX_PORT       GPIOC

/* UART4 */
#define UART4_TX_PIN        GPIO_PIN_0
#define UART4_TX_PORT       GPIOA
#define UART4_RX_PIN        GPIO_PIN_1
#define UART4_RX_PORT       GPIOA

/* UART5 */
#define UART5_TX_PIN        GPIO_PIN_12
#define UART5_TX_PORT       GPIOC
#define UART5_RX_PIN        GPIO_PIN_2
#define UART5_RX_PORT       GPIOD

/* UART6 */
#define UART6_TX_PIN        GPIO_PIN_6
#define UART6_TX_PORT       GPIOC
#define UART6_RX_PIN        GPIO_PIN_7
#define UART6_RX_PORT       GPIOC

/* I2C2 (Magnetometer, Barometer) */
#define I2C2_SCL_PIN        GPIO_PIN_10
#define I2C2_SCL_PORT       GPIOB
#define I2C2_SDA_PIN        GPIO_PIN_11
#define I2C2_SDA_PORT       GPIOB

/* SPI1 (Gyro/Accelerometer) */
#define SPI1_SCK_PIN        GPIO_PIN_5
#define SPI1_SCK_PORT       GPIOA
#define SPI1_MISO_PIN       GPIO_PIN_6
#define SPI1_MISO_PORT      GPIOA
#define SPI1_MOSI_PIN       GPIO_PIN_7
#define SPI1_MOSI_PORT      GPIOA

/* SPI2 (OSD, SD Card) */
#define SPI2_SCK_PIN        GPIO_PIN_13
#define SPI2_SCK_PORT       GPIOB
#define SPI2_MISO_PIN       GPIO_PIN_14
#define SPI2_MISO_PORT      GPIOB
#define SPI2_MOSI_PIN       GPIO_PIN_15
#define SPI2_MOSI_PORT      GPIOB

/* Chip Select Pins */
#define GYRO_CS_PIN         GPIO_PIN_4
#define GYRO_CS_PORT        GPIOA
#define OSD_CS_PIN          GPIO_PIN_12
#define OSD_CS_PORT         GPIOB
#define SDCARD_CS_PIN       GPIO_PIN_15
#define SDCARD_CS_PORT      GPIOA

/* Interrupt Pins */
#define GYRO_EXTI_PIN       GPIO_PIN_4
#define GYRO_EXTI_PORT      GPIOC

/* ADC Channels */
#define ADC_BATT_PIN        GPIO_PIN_0
#define ADC_BATT_PORT       GPIOC
#define ADC_CURR_PIN        GPIO_PIN_1
#define ADC_CURR_PORT       GPIOC
#define ADC_RSSI_PIN        GPIO_PIN_2
#define ADC_RSSI_PORT       GPIOC

/* Camera Control */
#define CAMERA_CONTROL_PIN  GPIO_PIN_3
#define CAMERA_CONTROL_PORT GPIOB

/* PPM Input */
#define PPM_PIN             GPIO_PIN_3
#define PPM_PORT            GPIOA

/* GPIO Output (PINIO) */
#define PINIO_1_PIN         GPIO_PIN_3
#define PINIO_1_PORT        GPIOC

/* ========== Timer Assignments ========== */
#define MOTOR_1_TIMER       TIM4
#define MOTOR_1_CHANNEL     TIM_CHANNEL_1
#define MOTOR_1_AF          GPIO_AF2_TIM4

#define MOTOR_2_TIMER       TIM4
#define MOTOR_2_CHANNEL     TIM_CHANNEL_2
#define MOTOR_2_AF          GPIO_AF2_TIM4

#define MOTOR_3_TIMER       TIM4
#define MOTOR_3_CHANNEL     TIM_CHANNEL_3
#define MOTOR_3_AF          GPIO_AF2_TIM4

#define MOTOR_4_TIMER       TIM4
#define MOTOR_4_CHANNEL     TIM_CHANNEL_4
#define MOTOR_4_AF          GPIO_AF2_TIM4

#define MOTOR_5_TIMER       TIM3
#define MOTOR_5_CHANNEL     TIM_CHANNEL_3
#define MOTOR_5_AF          GPIO_AF2_TIM3

#define MOTOR_6_TIMER       TIM3
#define MOTOR_6_CHANNEL     TIM_CHANNEL_4
#define MOTOR_6_AF          GPIO_AF2_TIM3

#define MOTOR_7_TIMER       TIM3
#define MOTOR_7_CHANNEL     TIM_CHANNEL_2
#define MOTOR_7_AF          GPIO_AF2_TIM3

#define MOTOR_8_TIMER       TIM3
#define MOTOR_8_CHANNEL     TIM_CHANNEL_1
#define MOTOR_8_AF          GPIO_AF2_TIM3

#define SERVO_1_TIMER       TIM1
#define SERVO_1_CHANNEL     TIM_CHANNEL_1
#define SERVO_1_AF          GPIO_AF1_TIM1

#define LED_STRIP_TIMER     TIM8
#define LED_STRIP_CHANNEL   TIM_CHANNEL_4
#define LED_STRIP_AF        GPIO_AF3_TIM8

#define CAMERA_TIMER        TIM2
#define CAMERA_CHANNEL      TIM_CHANNEL_2
#define CAMERA_AF           GPIO_AF1_TIM2

#define PPM_TIMER           TIM5
#define PPM_CHANNEL         TIM_CHANNEL_4
#define PPM_AF              GPIO_AF2_TIM5

/* ========== Peripheral Bus Assignments ========== */
#define GYRO_SPI_BUS        SPI1
#define OSD_SPI_BUS         SPI2
#define SDCARD_SPI_BUS      SPI2

#define MAG_I2C_BUS         I2C2
#define BARO_I2C_BUS        I2C2

/* ========== Hardware Configuration ========== */
//#define HSE_VALUE           8000000U  /* 8 MHz external crystal */

/* IMU Configuration */
#define USE_GYRO_BMI270
#define USE_ACC_BMI270

/* OSD Configuration */
#define USE_MAX7456

/* Barometer Configuration */
#define USE_BARO_DPS310

#endif /* SPEEDYBEEF405V3_PINOUT_H */
