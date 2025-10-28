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
PID_t ATTITUDE_PID_COEFFS = {100.0f, 0.0f, 0.1f};
PID_t RATE_PID_COEFFS = {1.5f, 0.5f, 0.05f};
////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID TUNING COEFFICIENTS
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
// function that calculates PID output based on comparing the setpoint vs the current state

// this controller calculates the desired angular velocity based on attitude error
// input: desired attitude (quaternion) and current attitude (quaternion)
// outpit: desired angular velocity
angular_velocity_t calculateAttitudePID(Quaternion_vector_t setpoint, Quaternion_vector_t current_state, long time, PID_errors_t *error_storage) {  
    float current_time = time;
    float delta_time = current_time - error_storage->previous_time;

    // prevent division by zero
    if (delta_time < 0.001f) {
        delta_time = 0.001f;
    }

    error_storage->previous_time = current_time;

    // step 1: calculate the quaterion error
    // q_error = q_setpoint * q_current^(-1)
    Quaternion_vector_t current_conj = quaternionConjugate(current_state);
    Quaternion_vector_t q_error = quaternionMultiply(setpoint, current_conj);
    // normalize the error quaternion
    q_error = quaternionNormalize(q_error);

    // step 2: write the vector part of error quaternion for PID control
    // this approximates the angular error
    error_storage->current_error = q_error;

    // step 3: calculate proportional term
    float P_roll    = ATTITUDE_PID_COEFFS.kp * q_error.i;
    float P_pitch   = ATTITUDE_PID_COEFFS.kp * q_error.j;
    float P_yaw     = ATTITUDE_PID_COEFFS.kp * q_error.k;

    // step 4: calculate the derivative term
    float derivative_roll   = (q_error.i - error_storage->previous_error.i) / delta_time;
    float derivative_pitch  = (q_error.j - error_storage->previous_error.j) / delta_time;
    float derivative_yaw    = (q_error.k - error_storage->previous_error.k) / delta_time;
    float D_roll            = ATTITUDE_PID_COEFFS.kd * derivative_roll;
    float D_pitch           = ATTITUDE_PID_COEFFS.kd * derivative_pitch;
    float D_yaw             = ATTITUDE_PID_COEFFS.kd * derivative_yaw;

    // step 5: update the previous errors for the next iteration
    error_storage->previous_error = q_error;

    angular_velocity_t desired_rate;
    desired_rate.roll_rate  = P_roll    + D_roll;
    desired_rate.pitch_rate = P_pitch   + D_pitch;
    desired_rate.yaw_rate   = P_yaw     + D_yaw;


    return desired_rate;
}

// this controller calculates motor corrections based on angular velocity error
// input: desired angular velocity and current angular velocity from gyro
// output: motor throttle corrections
orientation_correction_t calculateRatePID(angular_velocity_t desired_rate, angular_velocity_t current_rate, float time, PID_rate_errors_t *error_storage) {
    float current_time = time;
    float delta_time = current_time - error_storage->previous_time;

    if (delta_time < 0.001f) {
        delta_time = 0.001f;
    }

    error_storage->previous_time = current_time;

    // step 1: calculate angular v errors from the gyroscope data
    float error_roll = desired_rate.roll_rate - current_rate.roll_rate;
    float error_pitch = desired_rate.pitch_rate - current_rate.pitch_rate;
    float error_yaw = desired_rate.yaw_rate - current_rate.yaw_rate;
    error_storage->current_error_roll = error_roll;
    error_storage->current_error_pitch = error_pitch;
    error_storage->current_error_yaw = error_yaw;

    // step 2 proportional term
    float P_roll = RATE_PID_COEFFS.kp * error_roll;
    float P_pitch = RATE_PID_COEFFS.kp * error_pitch;
    float P_yaw = RATE_PID_COEFFS.kp * error_yaw;

    // step 3: integral term
    float integral_deadzone = 0.02f;
    float I_roll, I_pitch, I_yaw = 0.0f;
    if (fabsf(error_roll) < integral_deadzone) {
        error_storage->integral_roll = 0.0f;
    } else {
        error_storage->integral_roll += error_roll * delta_time;
        I_roll = RATE_PID_COEFFS.ki * error_storage->integral_roll;
    }
    if (fabsf(error_pitch) < integral_deadzone) {
        error_storage->integral_pitch = 0.0f;
    } else {
        error_storage->integral_pitch += error_pitch * delta_time;
        I_pitch = RATE_PID_COEFFS.ki * error_storage->integral_pitch;
    }
    if (fabsf(error_yaw) < integral_deadzone) {
        error_storage->integral_yaw = 0.0f;
    } else {
        error_storage->integral_yaw += error_yaw * delta_time;
        I_yaw = RATE_PID_COEFFS.ki * error_storage->integral_yaw;
    }

    // step 4: derivative term
    float derivative_roll = (error_roll - error_storage->previous_error_roll) / delta_time;
    float derivative_pitch = (error_pitch - error_storage->previous_error_pitch) / delta_time;
    float derivative_yaw = (error_yaw - error_storage->previous_error_yaw) / delta_time;
    float D_roll = RATE_PID_COEFFS.kd * derivative_roll;
    float D_pitch = RATE_PID_COEFFS.kd * derivative_pitch;
    float D_yaw = RATE_PID_COEFFS.kd * derivative_yaw;

    // step 5: update previous errors
    error_storage->previous_error_roll = error_roll;
    error_storage->previous_error_pitch = error_pitch;
    error_storage->previous_error_yaw = error_yaw;

    // step 6: calc final output motoro corrections
    orientation_correction_t output;
    output.roll = P_roll + I_roll + D_roll;
    output.pitch = P_pitch + I_pitch + D_pitch;
    output.yaw = P_yaw + I_yaw + D_yaw;

    return output;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END PID LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////