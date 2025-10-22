#include <stdio.h>
#include <stdbool.h>
#include "pid.h"
#include "flight_control.h"
#include <unistd.h>

void updateThrottleFromPID(motor_throttle_states_t *mts, orientation_correction_t oc) {
    mts->front_left     +=  -oc.pitch + oc.roll - oc.yaw;
    mts->front_right    +=   oc.pitch + oc.roll + oc.yaw;
    mts->rear_left      +=   oc.pitch - oc.roll - oc.yaw;
    mts->rear_right     +=  -oc.pitch - oc.roll + oc.yaw;
}

int main() {
    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool system_fail = false;
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // throttle_states holds the initial throttle values sent to the motors. Each value inside of
    // this variable should be changed by the PID control so each motor has a different throttlel
    // value that corresponds to its correction state (i.e front motors are higher than rear if 
    // the drone is tilted too far forward)
    motor_throttle_states_t throttle_states = {0}; // sets all throttle values to 0? by default at start

    // current_attitude holds the current vector rotation (represented by quaternions) as read
    // by the sensors on the drone
    Quaternion_vector_t current_attitude = {1.0f, 0.0f, 0.0f, 0.0f}; // sets the default position as having zero rotation about the world axis

    // setpoint holds the current goal setpoint for each iteration, i.e. what vector angle the drone wants to achieve
    Quaternion_vector_t setpoint_attitude = {1.0f, 0.0f, 0.0f, 0.0f}; // sets the default setpoint as having zero rotation about the world axis (level)
    
    // flight control loop
    while (!system_fail) {
        // at the beginning, we start with what the initial throttle input value is
        setThrottle(100, &throttle_states); // sets all values in throttle_states to a value

        // creates a variable that holds the roll, pitch, and yaw correction values based on the PID calculation
        orientation_correction_t correction_factors = calculatePid(setpoint_attitude, current_attitude);

        // applies the correction to the current throttle values
        // this transforms throttle_states into 4 different speeds that work towards the setpoint
        updateThrottleFromPID(&throttle_states, correction_factors);

        printf("Front Left: %d | Front Right: %d | Rear Left: %d | Rear Right: %d\n",
            (int)throttle_states.front_left,
            (int)throttle_states.front_right,
            (int)throttle_states.rear_left,
            (int)throttle_states.rear_right);

        sleep(1);
    }
    
    return 0;
}