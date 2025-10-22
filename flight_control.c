#include "flight_control.h"
#include "pid.h"

// this function changes all values of the throttle_states variable to the same
// number, this should be used when recieving a packet from the user when they 
// set a throttle value (recieved radio frequency for example)
void setThrottle(uint8_t throttle) {
    throttle_states.front_left = throttle;
    throttle_states.front_right = throttle;
    throttle_states.rear_left = throttle;
    throttle_states.rear_right = throttle;
}