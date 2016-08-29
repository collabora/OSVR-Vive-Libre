/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Sensor Fusion */

#pragma once

#include <Eigen/Geometry>

#define FILTER_QUEUE_MAX_SIZE 256
typedef struct {
    int at, size;
    Eigen::Vector3d elems[FILTER_QUEUE_MAX_SIZE];
} vl_queue;

void vl_filter_queue_init(vl_queue* me, int size);
void vl_filter_queue_add(vl_queue* me, const Eigen::Vector3d *vec);
Eigen::Vector3d vl_filter_queue_get_mean(const vl_queue* me);

typedef struct {
	int state;

    Eigen::Quaterniond orientation;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;

	int iterations;

	// filter queues for magnetometer, accelerometers and angular velocity
    vl_queue fq_acceleration;
    vl_queue fq_angular_velocity;

	// gravity correction
    int device_level_count;
    double grav_error_angle;
    Eigen::Vector3d grav_error_axis;
    double grav_gain; // amount of correction

} vl_fusion;

void vl_fusion_init(vl_fusion* me);
void vl_fusion_update(vl_fusion* me, float dt, Eigen::Vector3d vec3_gyro, Eigen::Vector3d vec3_accel);
