#pragma once

#include <Eigen/Geometry>
#include <osvr/PluginKit/TrackerInterfaceC.h>

static inline Eigen::Quaterniond switch_coord_order(Eigen::Quaterniond* in) {
    Eigen::Quaterniond q(in->x(), in->y(), in->z(), in->w());
    return q;
}

static inline Eigen::Quaterniond eigen_quaternion_inverse_handedness(Eigen::Quaterniond in) {
    Eigen::Quaterniond q(in.w(), -in.x(), -in.y(), -in.z());
    return q;
}

static inline void print_eigen_quat(const char* label, Eigen::Quaterniond* in) {
    printf("%s: %f %f %f %f\n", label, in->w(), in->x(), in->y(), in->z());
}
