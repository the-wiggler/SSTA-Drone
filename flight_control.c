#include "flight_control.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
// QUATERNION LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////

// quaternion conjugate (inverse for unit quaternions)
// to calculate the error between two orientations, you need to "undo" the current rotation and apply the desired rotation. 
// the conjugate represents that "undo" operation
Quaternion_vector_t quaternionConjugate(Quaternion_vector_t q) {
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
Quaternion_vector_t quaternionMultiply(Quaternion_vector_t q1, Quaternion_vector_t q2) {
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
Quaternion_vector_t quaternionNormalize(Quaternion_vector_t q) {
    float norm = sqrtf(q.w * q.w + q.i * q.i + q.j * q.j + q.k * q.k);
    
    q.w /= norm;
    q.i /= norm;
    q.j /= norm;
    q.k /= norm;
    
    return q;
}

// this function converts euler angles attitude vectors to quaternion vectors
// the function should be used to take an euler rotation and write that rotation to a specific 
// quaternion vector array. For example, if I got a rotation angle from a gyroscope I could pass them
// into this function to convert those rotations to my quaternion vector array
void writeAngleToVector(float roll, float pitch, float yaw, Quaternion_vector_t *vec) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    vec->w = cr * cp * cy + sr * sp * sy;
    vec->i = sr * cp * cy - cr * sp * sy;
    vec->j = cr * sp * cy + sr * cp * sy;
    vec->k = cr * cp * sy - sr * sp * cy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// END QUATERNION LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
// THROTTLE LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
// this function changes all values of the throttle_states variable to the same
// number, this should be used when recieving a packet from the user when they 
// set a throttle value (recieved radio frequency for example)
void setThrottle(throttle_t throttle, motor_throttle_states_t *mts) {
    mts->front_left     = throttle;
    mts->front_right    = throttle;
    mts->rear_left      = throttle;
    mts->rear_right     = throttle;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// END THROTTLE LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////////
