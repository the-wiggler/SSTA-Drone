#include "pid.h"
#include <string.h>
#include <math.h>


////////////////////////////////////////////////////////////////////////////////////////////////////

// PID tuning coefficients
PID_t roll_attitude_pid_coeffs  = { 50, 5, 17 };
PID_t pitch_attitude_pid_coeffs = { 50, 5, 17 };
PID_t yaw_attitude_pid_coeffs   = { 10, 2, 5 };

// PID error state storage
// this stores the PID error values that we use to know "how much" we should correct the motors
PID_errors_t roll_attitude_errors   = { 0, 0, 0, 0 };
PID_errors_t pitch_attitude_errors  = { 0, 0, 0, 0 };
PID_errors_t yaw_attitude_errors    = { 0, 0, 0, 0 };

// function that calculates PID output based on comparing the setpoint vs the current state
float calculate_pid(const PID_t *pid_coeffs, PID_errors_t *errors, float setpoint, 
                    float current_state) {
    // get the current time
    float current_time = get_time(); // this function doesnt exist

    // calculate the change in time
    float delta_time = (current_time - errors->previous_time);

    // calculates the current absolute error from the setpoint
    // proportional term
    errors->current_error = setpoint - current_state;

    // calculate integral term
    errors->integral += errors->current_error * delta_time;

    // calculate derivative term
    float derivative = 0;
    if (delta_time > 0) {
        derivative = (errors->current_error - errors->previous_error) / delta_time;
    }

    // calculate PID output using the formula (look on google for "pid formula")
    float output = (pid_coeffs->kp * errors->current_error) +
                      (pid_coeffs->ki * errors->integral) +
                      (pid_coeffs->kd * derivative);
    
    // update for the next iteration
    errors->previous_error = errors->current_error;
    errors->previous_time = current_time;

    // returns final PID output value that was calculated above
    return output;
}

int16_t get_time() {
    return 100;
}