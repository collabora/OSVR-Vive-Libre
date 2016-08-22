#pragma once

#include <Eigen/Geometry>
#include <osvr/PluginKit/TrackerInterfaceC.h>

#include "omath.h"

static inline vec3f vec3_eigen_to_ohmd(Eigen::Vector3f v) {
    vec3f vector;
    vector.x = v.x();
    vector.y = v.y();
    vector.z = v.z();
}

static inline Eigen::Quaternionf openhmd_to_eigen_quaternion(quatf in) {
    Eigen::Quaternionf q(in.x, in.y, in.z, in.w);
    return q;
}

static inline OSVR_Quaternion openhmd_to_osvr_quaternion(quatf in) {
    OSVR_Quaternion quat;
    quat.data[0] = in.x;
    quat.data[1] = in.y;
    quat.data[2] = in.z;
    quat.data[3] = in.w;
    return quat;
}

static inline OSVR_Quaternion eigen_to_osvr_quaternion(Eigen::Quaternionf in) {
    OSVR_Quaternion quat;
    quat.data[0] = in.w();
    quat.data[1] = -in.x();
    quat.data[2] = -in.y();
    quat.data[3] = -in.z();
    return quat;
}
