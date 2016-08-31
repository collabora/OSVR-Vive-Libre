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

void vl_fusion_init(vl_fusion* me)
{
    memset(me, 0, sizeof(vl_fusion));

    me->orientation = Eigen::Quaterniond(1,0,0,0);
    //me->orientation = Eigen::Quaterniond(0.108076, -0.669340, 0.113456, 0.726245);

    vl_filter_queue_init(&me->fq_acceleration, 20);
    vl_filter_queue_init(&me->fq_angular_velocity, 20);

    me->grav_gain = 0.05f;
}

#define GRAVITY_EARTH 9.82f

double get_angle(const Eigen::Vector3d* me, const Eigen::Vector3d* vec)
{
    double lengths = me->norm() * vec->norm();
    if (lengths == 0)
        return 0;
    return acos(me->dot(*vec) / lengths);
}

Eigen::Quaterniond* quat_init_axis(const Eigen::Vector3d* vec, double angle)
{
    return new Eigen::Quaterniond(
                cos(angle / 2.0f),
                vec->x() * sin(angle / 2.0f),
                vec->y() * sin(angle / 2.0f),
                vec->z() * sin(angle / 2.0f));
}

Eigen::Quaterniond* correct_gravity(vl_fusion* me, const Eigen::Vector3d* acceleration, float ang_vel_length) {
    const double gravity_tolerance = .4f, ang_vel_tolerance = .1f;
    const double min_tilt_error = 0.05f, max_tilt_error = 0.01f;

    // if the device is within tolerance levels, count this as the device is level and add to the counter
    // otherwise reset the counter and start over
    if (ang_vel_length < ang_vel_tolerance && fabsf(acceleration->norm() - GRAVITY_EARTH) < gravity_tolerance)
        me->device_level_count++;


    // device has been level for long enough, grab mean from the accelerometer filter queue (last n values)
    // and use for correction
    if(me->device_level_count > 50){
        me->device_level_count = 0;

        Eigen::Vector3d acceleration_mean = vl_filter_queue_get_mean(&me->fq_acceleration);
        acceleration_mean.normalize();

        // Calculate a cross product between what the device
        // thinks is up and what gravity indicates is down.
        // The values are optimized of what we would get out
        // from the cross product.
        Eigen::Vector3d up = Eigen::Vector3d::UnitY();
        double tilt_angle = get_angle(&up, &acceleration_mean);

        if(tilt_angle > max_tilt_error){
            Eigen::Vector3d tilt_e = Eigen::Vector3d(
                        acceleration_mean.z(),
                        0,
                        -acceleration_mean.x());
            tilt_e.normalize();

            me->grav_error_angle = tilt_angle;
            me->grav_error_axis = tilt_e;
        }
    }

    // preform gravity tilt correction
    if(me->grav_error_angle > min_tilt_error){
        double use_angle;
        if(me->grav_error_angle > gravity_tolerance && me->iterations < 2000){
            // if less than 2000 iterations have passed, set the up axis to the correction value outright
            use_angle = -me->grav_error_angle;
            me->grav_error_angle = 0;
        } else {
            // otherwise try to correct
            use_angle = -me->grav_gain * me->grav_error_angle * 0.005f * (5.0f * ang_vel_length + 1.0f);
            me->grav_error_angle += use_angle;
        }

        // perform the correction
        return quat_init_axis(&me->grav_error_axis, use_angle);
    }

    return NULL;
}

void vl_filter_queue_init(vl_queue* me, int size)
{
    memset(me, 0, sizeof(vl_queue));
    me->size = size;
}

void vl_filter_queue_add(vl_queue* me, const Eigen::Vector3d* vec)
{
    me->elems[me->at] = *vec;
    int x = me->at + 1;
    me->at = x % me->size;
}

Eigen::Vector3d vl_filter_queue_get_mean(const vl_queue* me)
{
    Eigen::Vector3d mean;
    for(int i = 0; i < me->size; i++)
        mean += me->elems[i];
    return mean / (double)me->size;
}


void vl_fusion_update(vl_fusion* me, float dt, Eigen::Vector3d angular_velocity, Eigen::Vector3d acceleration)
{
    Eigen::Vector3d acceleration_world = me->orientation * acceleration;

    me->iterations += 1;

    vl_filter_queue_add(&me->fq_acceleration, &acceleration_world);
    vl_filter_queue_add(&me->fq_angular_velocity, &angular_velocity);

    float ang_vel_length = angular_velocity.norm();

    if (ang_vel_length > 0.0001f) {
        Eigen::Vector3d rot_axis = angular_velocity.normalized();
        //Eigen::Vector3d rot_axis = -angular_velocity.normalized();
        float rot_angle = ang_vel_length * dt;
        me->orientation = me->orientation * Eigen::AngleAxisd(rot_angle, rot_axis);
    }

    // gravity correction
    Eigen::Quaterniond* correction = correct_gravity(me, &acceleration, ang_vel_length);
    if (correction != NULL) {
        me->orientation = *correction * me->orientation;
        delete(correction);
    }

    // mitigate drift due to floating point
    // inprecision with quat multiplication.
    me->orientation.normalize();
}
