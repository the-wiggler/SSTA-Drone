#include <stdio.h>
#include <stdbool.h>
#include "pid.h"
#include "flight_control.h"
#include <unistd.h>
#include <math.h>
#include <time.h>
//NOTE: ALL COORDINATE SYSTEM AXES ARE DEFINED AS FOLLOWS FOR THIS DRONE:
// AS VIEWED FROM THE DIRECT BACK OF THE DRONE
// THE POSITIVE ROLL AXIS POINTS THROUGH TOWARDS THE FRONT OF THE DRONE
// THE POSITIVE PITCH AXIS POINTS RIGHTWARD
// THE POSITIVE YAW AXIS POINTS UPWARD

// NOTE: ROTATIONS ARE DEFINED AS FOLLOWS:
// rolling clockwise is positive, pitching nose up is positive, yawing left is positive

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
// END HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
// PID error state storage
PID_errors_t attitude_errors = {
    .current_error  = {1.0f, 0.0f, 0.0f, 0.0f},
    .previous_error = {1.0f, 0.0f, 0.0f, 0.0f},
    .integral       = {0.0f, 0.0f, 0.0f, 0.0f},
    .previous_time  = 0.0f
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

    // test to set a different roll angle than the setpoint:
    float pitch_angle = 0.1745f; // +10 degrees in radians (pitch forward/down)

    current_attitude.w = cosf(pitch_angle / 2.0f);
    current_attitude.i = 0.0f;  // no roll
    current_attitude.j = sinf(pitch_angle / 2.0f);  // pitch rotation
    current_attitude.k = 0.0f;  // no yaw

    // setpoint holds the current goal setpoint for each iteration, i.e. what vector angle the drone wants to achieve
    Quaternion_vector_t setpoint_attitude = {1.0f, 0.0f, 0.0f, 0.0f}; // sets the default setpoint as having zero rotation about the world axis (level)
    
    // flight control loop
    while (!system_fail) {
        // at the beginning, we start with what the initial throttle input value is
        setThrottle(100, &throttle_states); // sets all values in throttle_states to a value

        // creates a variable that holds the roll, pitch, and yaw correction values based on the PID calculation
        orientation_correction_t correction_factors = calculatePid(setpoint_attitude, current_attitude, timeSinceStart(), attitude_errors);

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