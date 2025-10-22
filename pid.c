////////////////////////////////////////////////////////////////////////////////////////////////////
// REFERENCE INFORMATION
////////////////////////////////////////////////////////////////////////////////////////////////////

// Quaternion multiplication rules:
// i^2 = j^2 = k^2 = -1
// ij = -ji = k
// jk = -kj = i
// ki = -ki = j

////////////////////////////////////////////////////////////////////////////////////////////////////
// END REFERENCE INFORMATION
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "pid.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID TUNING COEFFICIENTS
////////////////////////////////////////////////////////////////////////////////////////////////////
PID_t PID_COEFFS = {50, 0, 0};


// PID error state storage
PID_errors_t attitude_errors = {
    .current_error  = {1.0f, 0.0f, 0.0f, 0.0f},
    .previous_error = {1.0f, 0.0f, 0.0f, 0.0f},
    .integral       = {0.0f, 0.0f, 0.0f, 0.0f},
    .previous_time  = 0.0f
};
////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID TUNING COEFFICIENTS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
// function that calculates PID output based on comparing the setpoint vs the current state
// format:
// current state: the current attitude of the drone as read by the sensors
// desired orientation (setpoint): the attitude we're trying to achieve (0 if just trying to hover)
// error: the rotation (like distance) needed to get from current to desired 

// this function returns an orientation struct type that contains three values: roll, pitch, and yaw corrections
// these values represent the PID controller's calculated corrections for the drone's orientation along each axis
// these values should be either added or subtracted from the desired throttle based on the following parameters to achieve the setpoint:
//      front-right motor: throttle - pitch + roll - yaw
//      rear-right motor: throttle + pitch + roll + yaw
//      rear-left motor: throttle + pitch - roll - yaw
//      front-left motor: throttle - pitch - roll + yaw
orientation_correction_t calculatePid(Quaternion_vector_t setpoint, Quaternion_vector_t current_state) {  
    float current_time = get_time();
    float delta_time = current_time - attitude_errors.previous_time;
    attitude_errors.previous_time = current_time;

    // step 1: calculate the quaterion error
    // q_error = q_setpoint * q_current^(-1)
    Quaternion_vector_t current_conj = quaternion_conjugate(current_state);
    Quaternion_vector_t q_error = quaternion_multiply(setpoint, current_conj);
    // normalize the error quaternion
    q_error = quaternion_normalize(q_error);

    // step 2: write the vector part of error quaternion for PID control
    // this approximates the angular error
    attitude_errors.current_error = q_error;

    // step 3: calculate proportional term
    float P_i = PID_COEFFS.kp * q_error.i;
    float P_j = PID_COEFFS.kp * q_error.j;
    float P_k = PID_COEFFS.kp * q_error.k;

    // step 4: calculate integral term
    // ntegrate error over time
    attitude_errors.integral.i += q_error.i * delta_time;
    attitude_errors.integral.j += q_error.j * delta_time;
    attitude_errors.integral.k += q_error.k * delta_time;
    float I_i = PID_COEFFS.ki * attitude_errors.integral.i;
    float I_j = PID_COEFFS.ki * attitude_errors.integral.j;
    float I_k = PID_COEFFS.ki * attitude_errors.integral.k;

    // step 5: calculate the derivative term
    float derivative_i = (q_error.i - attitude_errors.previous_error.i) / delta_time;
    float derivative_j = (q_error.j - attitude_errors.previous_error.j) / delta_time;
    float derivative_k = (q_error.k - attitude_errors.previous_error.k) / delta_time;
    float D_i = PID_COEFFS.kd * derivative_i;
    float D_j = PID_COEFFS.kd * derivative_j;
    float D_k = PID_COEFFS.kd * derivative_k;

    // step 6: update the previous errors for the next iteration
    attitude_errors.previous_error = q_error;

    orientation_correction_t output;
    output.roll = P_i + I_i + D_i;   // roll
    output.pitch = P_j + I_j + D_j;  // pirch
    output.yaw = P_k + I_k + D_k;    // yaw

    return output;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////


int16_t get_time() {
    return 100;
}