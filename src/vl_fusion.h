/*
 * OpenHMD
 *
 * Copyright (C) 2013 Fredrik Hultin
 * Copyright (C) 2013 Jakob Bornecrantz
 *
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
#include <list>
#include <mutex>

#define FILTER_QUEUE_MAX_SIZE 256
class vl_filter_queue {
    unsigned size;
    //Eigen::Vector3d elems[FILTER_QUEUE_MAX_SIZE];
    std::list<Eigen::Vector3d> queue;
    std::mutex mutex;

public:
    vl_filter_queue(int size);
    ~vl_filter_queue();
    void add(Eigen::Vector3d vec);
    Eigen::Vector3d get_mean();
};

class vl_fusion {
private:
	int state;

    std::mutex mutex_fusion_update;

    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;

	int iterations;

	// filter queues for magnetometer, accelerometers and angular velocity
    vl_filter_queue* fq_acceleration;
    vl_filter_queue* fq_angular_velocity;

	// gravity correction
    int device_level_count;
    double grav_error_angle;
    Eigen::Vector3d grav_error_axis;
    double grav_gain; // amount of correction

    Eigen::Quaterniond* correct_gravity(const Eigen::Vector3d* acceleration, float ang_vel_length);

public:
    Eigen::Quaterniond orientation;

    vl_fusion();
    ~vl_fusion();
    void update(float dt, const Eigen::Vector3d &vec3_gyro, const Eigen::Vector3d &vec3_accel);
};


