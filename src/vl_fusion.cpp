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

#include <string.h>
#include "vl_fusion.h"
#include "vl_math.h"

vl_fusion::vl_fusion() {
    orientation = Eigen::Quaterniond(0, -1, 0, 0);
    fq_acceleration = new vl_filter_queue(20);
    fq_angular_velocity = new vl_filter_queue(20);
    grav_gain = 0.05f;
}

vl_fusion::~vl_fusion() {
    delete(fq_acceleration);
    delete(fq_angular_velocity);
}


#define GRAVITY_EARTH 9.82f


Eigen::Quaterniond* vl_fusion::correct_gravity(const Eigen::Vector3d* acceleration, float ang_vel_length) {
    const double gravity_tolerance = .4f, ang_vel_tolerance = .1f;
    const double min_tilt_error = 0.05f, max_tilt_error = 0.01f;

    // if the device is within tolerance levels, count this as the device is level and add to the counter
    // otherwise reset the counter and start over
    if (ang_vel_length < ang_vel_tolerance && fabsf(acceleration->norm() - GRAVITY_EARTH) < gravity_tolerance)
        device_level_count++;


    // device has been level for long enough, grab mean from the accelerometer filter queue (last n values)
    // and use for correction
    if(device_level_count > 50){
        device_level_count = 0;

        Eigen::Vector3d acceleration_mean = this->fq_acceleration->get_mean();
        acceleration_mean.normalize();

        // Calculate a cross product between what the device
        // thinks is up and what gravity indicates is down.
        // The values are optimized of what we would get out
        // from the cross product.
        Eigen::Vector3d up = Eigen::Vector3d::UnitY();
        double tilt_angle = vector_get_angle(&up, &acceleration_mean);

        if(tilt_angle > max_tilt_error){
            Eigen::Vector3d tilt_e = Eigen::Vector3d(
                        acceleration_mean.z(),
                        0,
                        -acceleration_mean.x());
            tilt_e.normalize();

            grav_error_angle = tilt_angle;
            grav_error_axis = tilt_e;
        }
    }

    // preform gravity tilt correction
    if(grav_error_angle > min_tilt_error){
        double use_angle;
        if(grav_error_angle > gravity_tolerance && iterations < 2000){
            // if less than 2000 iterations have passed, set the up axis to the correction value outright
            use_angle = -grav_error_angle;
            grav_error_angle = 0;
        } else {
            // otherwise try to correct
            use_angle = -grav_gain * grav_error_angle * 0.005f * (5.0f * ang_vel_length + 1.0f);
            grav_error_angle += use_angle;
        }

        // perform the correction
        return quat_init_axis(&this->grav_error_axis, use_angle);
    }

    return nullptr;
}

vl_filter_queue::~vl_filter_queue() {}
vl_filter_queue::vl_filter_queue(int size) {
    this->size = size;
}

void vl_filter_queue::add(Eigen::Vector3d vec)
{
    Eigen::Vector3d v = Eigen::Vector3d(vec.x(), vec.y(), vec.z());
    queue.push_back(v);
    if (queue.size() > size)
        queue.pop_front();
}
Eigen::Vector3d vl_filter_queue::get_mean()
{
    Eigen::Vector3d mean;
    for(Eigen::Vector3d vec : queue)
           mean += vec;
    return mean / (double)queue.size();
}


void vl_fusion::update(float dt, const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& acceleration)
{
    mutex_fusion_update.lock();

    Eigen::Vector3d acceleration_world = orientation * acceleration;

    iterations += 1;

    fq_acceleration->add(acceleration_world);
    fq_angular_velocity->add(angular_velocity);

    float ang_vel_length = angular_velocity.norm();

    if (ang_vel_length > 0.0001f) {
        Eigen::Vector3d rot_axis = angular_velocity.normalized();
        //Eigen::Vector3d rot_axis = -angular_velocity.normalized();
        float rot_angle = ang_vel_length * dt;
        orientation = orientation * Eigen::AngleAxisd(rot_angle, rot_axis);
    }

    // gravity correction
    Eigen::Quaterniond* correction = correct_gravity(&acceleration, ang_vel_length);
    if (correction != nullptr) {
        orientation = *correction * orientation;
        delete(correction);
    }

    // mitigate drift due to floating point
    // inprecision with quat multiplication.
    orientation.normalize();

    // print_eigen_quat("pose", &orientation);
    mutex_fusion_update.unlock();
}
