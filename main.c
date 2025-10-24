#include <stdio.h>
#include <time.h>
#include "pid.h"
#include "flight_control.h"
#include <stdbool.h>
#include <unistd.h>
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
static clock_t start_time;
void initTimer() {
    start_time = clock();
}
double timeSinceStart() {
    return ((double)(clock() - start_time)) / CLOCKS_PER_SEC;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
// PID error state storage
PID_errors_t attitude_errors = {
    .current_error  = {1.0f, 0.0f, 0.0f, 0.0f},
    .previous_error = {1.0f, 0.0f, 0.0f, 0.0f},
    .integral       = {0.0f, 0.0f, 0.0f, 0.0f},
    .previous_time  = 0.0f
};
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


int main() {
    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool system_fail = false;
    initTimer();
    ////////////////////////////////////////////////////////////////////////////////////////////////

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
    writeAngleToVector(0.2f, 0.0f, 0.0f, &current_attitude);
    
    // flight control loop
    while (!system_fail) {
        // at the beginning, we start with what the initial throttle input value is
        setThrottle(100, &throttle_states); // sets all values in throttle_states to a value

        // PD attitude correction
        angular_velocity_t desired_angular_v = calculateAttitudePID(setpoint_attitude, current_attitude, timeSinceStart(), &attitude_errors);

        // PID roll rate correction
        orientation_correction_t correction_factors = calculateRatePID(desired_angular_v, current_angular_v, timeSinceStart(), &rate_errors);

        // applies the correction to the current throttle values
        // this transforms throttle_states into 4 different speeds that work towards the setpoint
        updateThrottleFromPID(&throttle_states, correction_factors);

        printf("Front Left: %d | Front Right: %d | Rear Left: %d | Rear Right: %d\n",
            (int)throttle_states.front_left,
            (int)throttle_states.front_right,
            (int)throttle_states.rear_left,
            (int)throttle_states.rear_right);

        if (timeSinceStart() > 5.0f) writeAngleToVector(0.0f, 0.0f, 0.0f, &current_attitude);
        //sleep(1);
    }
    
    return 0;
}