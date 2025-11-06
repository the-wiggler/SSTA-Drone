#include "SpeedyBee_F405_conf.h"
#include <stdbool.h>
#include <stdint.h>
#include "pid.h"
#include "flight_control.h"
#include "debug.h"
#include <string.h>

//NOTE: ALL COORDINATE SYSTEM AXES ARE DEFINED AS FOLLOWS FOR THIS DRONE:
// AS VIEWED FROM THE DIRECT BACK OF THE DRONE
// THE POSITIVE ROLL AXIS POINTS THROUGH TOWARDS THE FRONT OF THE DRONE
// THE POSITIVE PITCH AXIS POINTS RIGHTWARD
// THE POSITIVE YAW AXIS POINTS UPWARD

// NOTE: ROTATIONS ARE DEFINED AS FOLLOWS:
// rolling clockwise is positive, pitching nose up is positive, yawing left is positive

// NOTE: YOU SHOULD BE USING RADIANS!!!

////////////////////////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
// TIME FUNCTIONS
// configure system clock
void SystemClock_Config(void);
static uint32_t start_time;
void initTimer() {
    start_time = HAL_GetTick();
}
double timeSinceStart() {
    return (double)(HAL_GetTick() - start_time) / 1000.0; // returns time in seconds
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// END HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARS
////////////////////////////////////////////////////////////////////////////////////////////////////

// stores the raw sensor data gathered by the onboard BMP270
BMP270_raw_data_t BMP270_sensor_data = {0};

// PID error state storage
PID_errors_t attitude_errors = {
    .current_error  = {1.0f, 0.0f, 0.0f, 0.0f},
    .previous_error = {1.0f, 0.0f, 0.0f, 0.0f},
    .integral       = {0.0f, 0.0f, 0.0f, 0.0f},
    .previous_time  = 0.0f
};
// PID error state storage
PID_rate_errors_t rate_errors = {
    .current_error_roll = 0.0f,
    .current_error_pitch = 0.0f,
    .current_error_yaw = 0.0f,
    .previous_error_roll = 0.0f,
    .previous_error_pitch = 0.0f,
    .previous_error_yaw = 0.0f,
    .integral_roll = 0.0f,
    .integral_pitch = 0.0f,
    .integral_yaw = 0.0f,
    .previous_time = 0.0f
};
////////////////////////////////////////////////////////////////////////////////////////////////////
// END GLOBAL VARS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main() {
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize HAL and other such system things
    HAL_Init();
    SystemClock_Config();
    bool system_fail = false;
    initTimer();
    FC_LEDInit();
    BMP270_SPIInit();
    debugInit();
    // throttle_states holds the initial throttle values sent to the motors. Each value inside of
    // this variable should be changed by the PID control so each motor has a different throttlel
    // value that corresponds to its correction state (i.e front motors are higher than rear if 
    // the drone is tilted too far forward)
    motor_throttle_states_t throttle_states = {0}; // sets all throttle values to 0? by default at start

    // current_attitude holds the current vector rotation (represented by quaternions) as read
    // by the sensors on the drone
    // use the right hand rule to determine the angles of orientation (rolling clockwise is positive, pitching nose up is positive, yawing left is positive)
    Quaternion_vector_t current_attitude = {1.0f, 0.0f, 0.0f, 0.0f}; // sets the default position as having zero rotation about the world axis
    // setpoint holds the current goal setpoint for each iteration, i.e. what vector angle the drone wants to achieve
    Quaternion_vector_t setpoint_attitude = {1.0f, 0.0f, 0.0f, 0.0f}; // sets the default setpoint as having zero rotation about the world axis (level)

    // current angular velocity from the gyroscope sensor
    // holds data in radians per second (rad/s)
    // these are NOT represented with quaternions, but roll, pitch, and yaw
    angular_velocity_t current_angular_v = {0.0f, 0.0f, 0.0f};

    // test to set a different roll angle than the setpoint to see the correction take place!
    //writeAngleToVector(0.2f, 0.0f, 0.0f, &current_attitude);
    
    // flight control loop
    while (!system_fail) {
        // before anything, sensor data from the onboard accelerometer and gyroscope are gathered
        BMP270_ReadSensorData(&BMP270_sensor_data);

        //HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        // at the beginning, we start with what the initial throttle input value is
        setThrottle(100, &throttle_states); // sets all values in throttle_states to a value

        // TO DO: read the actual sensor data here
        // current_attitude = readIMU();
        // current_angular_v = readGyro();

        // PD attitude correction
        angular_velocity_t desired_angular_v = calculateAttitudePID(setpoint_attitude, current_attitude, timeSinceStart(), &attitude_errors);

        // PID roll rate correction
        orientation_correction_t correction_factors = calculateRatePID(desired_angular_v, current_angular_v, timeSinceStart(), &rate_errors);

        // applies the correction to th e current throttle values
        // this transforms throttle_states into 4 different speeds that work towards the setpoint
        updateThrottleFromPID(&throttle_states, correction_factors);

        // TO DO: output motor values via PWM
        // setMotorPWM(throttle_states);
        debugPrint("Roll: %d  Pitch: %d  Yaw: %d\r\n", BMP270_sensor_data.omega_roll, BMP270_sensor_data.gyr_pitch, BMP270_sensor_data.gyr_yaw);

        HAL_Delay(500);
    }
    
    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// system clock config for STM32F405
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}