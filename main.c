#include <stdio.h>
#include <stdbool.h>
#include "pid.h"
#include "flight_control.h"

int main() {
    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool system_fail = false;
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // throttle_states holds the initial throttle values sent to the motors. Each value inside of
    // this variable should be changed by the PID control so each motor has a different throttlel
    // value that corresponds to its correction state (i.e front motors are higher than rear if 
    // the drone is tilted too far forward)
    motor_throttle_states_t throttle_states = {0}; // sets all throttle values to 0 by default at start

    setThrottle(100);
    
    // flight control loop
    while (!system_fail) {

    }
    
    return 0;
}