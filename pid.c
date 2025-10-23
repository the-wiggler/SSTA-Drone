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
PID_t PID_COEFFS = {120, 0, 0};
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
// these values should be either added or subtracted from the desired throttle based on an equation
orientation_correction_t calculatePid(Quaternion_vector_t setpoint, Quaternion_vector_t current_state, long time, PID_errors_t error_storage) {  
    float current_time = time;
    float delta_time = current_time - error_storage.previous_time;
    error_storage.previous_time = current_time;

    // step 1: calculate the quaterion error
    // q_error = q_setpoint * q_current^(-1)
    Quaternion_vector_t current_conj = quaternionConjugate(current_state);
    Quaternion_vector_t q_error = quaternionMultiply(setpoint, current_conj);
    // normalize the error quaternion
    q_error = quaternionNormalize(q_error);

    // step 2: write the vector part of error quaternion for PID control
    // this approximates the angular error
    error_storage.current_error = q_error;

    // step 3: calculate proportional term
    float P_i = PID_COEFFS.kp * q_error.i;
    float P_j = PID_COEFFS.kp * q_error.j;
    float P_k = PID_COEFFS.kp * q_error.k;

    // step 4: calculate integral term
    // ntegrate error over time
    error_storage.integral.i += q_error.i * delta_time;
    error_storage.integral.j += q_error.j * delta_time;
    error_storage.integral.k += q_error.k * delta_time;
    float I_i = PID_COEFFS.ki * error_storage.integral.i;
    float I_j = PID_COEFFS.ki * error_storage.integral.j;
    float I_k = PID_COEFFS.ki * error_storage.integral.k;

    // step 5: calculate the derivative term
    float derivative_i = (q_error.i - error_storage.previous_error.i) / delta_time;
    float derivative_j = (q_error.j - error_storage.previous_error.j) / delta_time;
    float derivative_k = (q_error.k - error_storage.previous_error.k) / delta_time;
    float D_i = PID_COEFFS.kd * derivative_i;
    float D_j = PID_COEFFS.kd * derivative_j;
    float D_k = PID_COEFFS.kd * derivative_k;

    // step 6: update the previous errors for the next iteration
    error_storage.previous_error = q_error;

    orientation_correction_t output;
    output.roll = P_i + I_i + D_i;   // roll
    output.pitch = P_j + I_j + D_j;  // pirch
    output.yaw = P_k + I_k + D_k;    // yaw

    return output;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////