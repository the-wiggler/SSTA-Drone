#include "pid.h"
#include <string.h>
#include <math.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////////////////////////
// PID tuning coefficients
PID_t PID_COEFFS = {50, 50, 15};


// PID error state storage
PID_errors_t attitude_errors = {
    .current_error  = {1.0f, 0.0f, 0.0f, 0.0f},
    .previous_error = {1.0f, 0.0f, 0.0f, 0.0f},
    .integral       = {0.0f, 0.0f, 0.0f, 0.0f},
    .previous_time  = 0.0f
};
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// QUATERNION LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////

// quaternion conjugate (inverse for unit quaternions)
// to calculate the error between two orientations, you need to "undo" the current rotation and apply the desired rotation. 
// the conjugate represents that "undo" operation
Quaternion_vector_t quaternion_conjugate(Quaternion_vector_t q) {
    Quaternion_vector_t q_conj;
    q_conj.w = q.w;
    q_conj.i = -q.i;
    q_conj.j = -q.j;
    q_conj.k = -q.k;
    return q_conj;
}


// quaternion multiplication
// result = q1 * q2
// this combines two rotations into one
// if you rotate by q1 then by q2, the combined rotation is q2×q1
Quaternion_vector_t quaternion_multiply(Quaternion_vector_t q1, Quaternion_vector_t q2) {
    Quaternion_vector_t result;
    
    result.w = q1.w * q2.w - q1.i * q2.i - q1.j * q2.j - q1.k * q2.k;
    result.i = q1.w * q2.i + q1.i * q2.w + q1.j * q2.k - q1.k * q2.j;
    result.j = q1.w * q2.j - q1.i * q2.k + q1.j * q2.w + q1.k * q2.i;
    result.k = q1.w * q2.k + q1.i * q2.j - q1.j * q2.i + q1.k * q2.w;
    
    return result;
}

// normalize quaternion
// sometimes the unit vectors get a bit off from 1 because floating point math is bad, so we have
// to fix that by normalizing them again
Quaternion_vector_t quaternion_normalize(Quaternion_vector_t q) {
    float norm = sqrtf(q.w * q.w + q.i * q.i + q.j * q.j + q.k * q.k);
    
    q.w /= norm;
    q.i /= norm;
    q.j /= norm;
    q.k /= norm;
    
    return q;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END QUATERNION LOGIC
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
orientation_correction calculate_pid(PID_t pid_coeffs, PID_errors_t *errors, Quaternion_vector_t setpoint, 
                                                                Quaternion_vector_t current_state) {  
    float current_time = get_time();
    float delta_time = current_time - errors->previous_time;
    errors->previous_time = current_time;

    // step 1: calculate the quaterion error
    // q_error = q_setpoint * q_current^(-1)
    Quaternion_vector_t current_conj = quaternion_conjugate(current_state);
    Quaternion_vector_t q_error = quaternion_multiply(setpoint, current_conj);
    // normalize the error quaternion
    q_error = quaternion_normalize(q_error);

    // step 2: write the vector part of error quaternion for PID control
    // this approximates the angular error
    errors->current_error = q_error;

    // step 3: calculate proportional term
    float P_i = pid_coeffs.kp * q_error.i;
    float P_j = pid_coeffs.kp * q_error.j;
    float P_k = pid_coeffs.kp * q_error.k;

    // step 4: calculate integral term
    // ntegrate error over time
    errors->integral.i += q_error.i * delta_time;
    errors->integral.j += q_error.j * delta_time;
    errors->integral.k += q_error.k * delta_time;
    float I_i = pid_coeffs.ki * errors->integral.i;
    float I_j = pid_coeffs.ki * errors->integral.j;
    float I_k = pid_coeffs.ki * errors->integral.k;

    // step 5: calculate the derivative term
    float derivative_i = (q_error.i - errors->previous_error.i) / delta_time;
    float derivative_j = (q_error.j - errors->previous_error.j) / delta_time;
    float derivative_k = (q_error.k - errors->previous_error.k) / delta_time;
    float D_i = pid_coeffs.kd * derivative_i;
    float D_j = pid_coeffs.kd * derivative_j;
    float D_k = pid_coeffs.kd * derivative_k;

    // step 6: update the previous errors for the next iteration
    errors->previous_error = q_error;

    orientation_correction output;
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