//==============================================================================
// quaternion.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef QUATERNION_H
#define QUATERNION_H

struct Quaterion_t {
    float w;
    float x;
    float y;
    float z;
};


inline Quaterion_t multiplyQuat(Quaterion_t q1, Quaterion_t q2) {
    Quaterion_t ret;
    
    ret.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    ret.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    ret.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    ret.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;

    return ret;
}

#endif
