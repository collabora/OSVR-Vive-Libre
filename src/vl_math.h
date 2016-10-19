/*
 * Vive Libre
 *
 * Copyright (C) 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */

#pragma once

#include <Eigen/Geometry>
#include "vl_log.h"

static inline void print_eigen_quat(const char* label, const Eigen::Quaterniond& in) {
    vl_info("%s: %f %f %f %f", label, in.w(), in.x(), in.y(), in.z());
}

static inline double vector_get_angle(const Eigen::Vector3d& me, const Eigen::Vector3d& vec) {
    double lengths = me.norm() * vec.norm();
    if (lengths == 0)
        return 0;
    return acos(me.dot(vec) / lengths);
}

static inline Eigen::Quaterniond* quat_init_axis(const Eigen::Vector3d& vec, double angle) {
    return new Eigen::Quaterniond(
                cos(angle / 2.0f),
                vec.x() * sin(angle / 2.0f),
                vec.y() * sin(angle / 2.0f),
                vec.z() * sin(angle / 2.0f));
}
