#include "pid.h"
#include <string.h>
#include <math.h>


////////////////////////////////////////////////////////////////////////////////////////////////////

// PID tuning coefficients
PID_t PID_COEFFS = {50, 50, 15};

// PID error state storage


// function that calculates PID output based on comparing the setpoint vs the current state
vector_t calculate_pid(PID_t pid_coeffs, PID_errors_t *errors, vector_t setpoint,
                       vector_t current_state) {
    vector_t outputs;
    
    float current_time = get_time();
    float delta_time = current_time - errors->previous_time;
    float inv_delta_time = 1.0f / delta_time;
    
    // process all axes
    float *errs[3] = {&errors->current_error.x, &errors->current_error.y, &errors->current_error.z};
    float *errs_prev[3] = {&errors->previous_error.x, &errors->previous_error.y, &errors->previous_error.z};
    float *integral[3] = {&errors->integral.x, &errors->integral.y, &errors->integral.z};
    float *proportional[3] = {&setpoint.x, &setpoint.y, &setpoint.z};
    float *cs[3] = {&current_state.x, &current_state.y, &current_state.z};
    float *out[3] = {&outputs.x, &outputs.y, &outputs.z};
    
    for(int i = 0; i < 3; i++) {
        // calculate error
        *errs[i] = *proportional[i] - *cs[i];
        
        // update integral
        *integral[i] += *errs[i] * delta_time;
        
        // calculate derivative
        float derivative = (*errs[i] - *errs[i]) * inv_delta_time;
        
        // calculate PID output
        *out[i] = pid_coeffs.kp * (*errs[i]) + 
                  pid_coeffs.ki * (*integral[i]) + 
                  pid_coeffs.kd * derivative;
        
        // update previous error
        *errs_prev[i] = *errs[i];
    }
    
    errors->previous_time = current_time;
    return outputs;
}


int16_t get_time() {
    return 100;
}