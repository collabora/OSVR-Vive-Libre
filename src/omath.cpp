/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Math Code Implementation */

#include <string.h>
#include "omath.h"

// vector

float ovec3f_get_length(const vec3f* me)
{
	return sqrtf(POW2(me->x) + POW2(me->y) + POW2(me->z));
}

void ovec3f_normalize_me(vec3f* me)
{
	if(me->x == 0 && me->y == 0 && me->z == 0)
		return;

	float len = ovec3f_get_length(me);
	me->x /= len;
	me->y /= len;
	me->z /= len;
}

float ovec3f_get_dot(const vec3f* me, const vec3f* vec)
{
	return me->x * vec->x + me->y * vec->y + me->z * vec->z;
}

float ovec3f_get_angle(const vec3f* me, const vec3f* vec)
{
	float dot = ovec3f_get_dot(me, vec);
	float lengths = ovec3f_get_length(me) * ovec3f_get_length(vec);

	if(lengths == 0)
		return 0;

	return acosf(dot / lengths);
}


// quaternion

void oquatf_init_axis(quatf* me, const vec3f* vec, float angle)
{
	vec3f norm = *vec;
	ovec3f_normalize_me(&norm);

	me->x = norm.x * sinf(angle / 2.0f);
	me->y = norm.y * sinf(angle / 2.0f);
	me->z = norm.z * sinf(angle / 2.0f);
	me->w = cosf(angle / 2.0f);
}

void oquatf_get_rotated(const quatf* me, const vec3f* vec, vec3f* out_vec)
{
	quatf q = {{vec->x * me->w + vec->z * me->y - vec->y * me->z,
	            vec->y * me->w + vec->x * me->z - vec->z * me->x,
	            vec->z * me->w + vec->y * me->x - vec->x * me->y,
	            vec->x * me->x + vec->y * me->y + vec->z * me->z}};

	out_vec->x = me->w * q.x + me->x * q.w + me->y * q.z - me->z * q.y;
	out_vec->y = me->w * q.y + me->y * q.w + me->z * q.x - me->x * q.z;
	out_vec->z = me->w * q.z + me->z * q.w + me->x * q.y - me->y * q.x;
}

void oquatf_mult(const quatf* me, const quatf* q, quatf* out_q)
{
    out_q->x = me->w * q->x + me->x * q->w + me->y * q->z - me->z * q->y;
    out_q->y = me->w * q->y - me->x * q->z + me->y * q->w + me->z * q->x;
    out_q->z = me->w * q->z + me->x * q->y - me->y * q->x + me->z * q->w;
    out_q->w = me->w * q->w - me->x * q->x - me->y * q->y - me->z * q->z;
}

void oquatf_mult_me(quatf* me, const quatf* q)
{
    quatf tmp = *me;
    oquatf_mult(&tmp, q, me);
}

void oquatf_normalize_me(quatf* me)
{
	float len = oquatf_get_length(me);
	me->x /= len;
	me->y /= len;
	me->z /= len;
	me->w /= len;
}

float oquatf_get_length(const quatf* me)
{
	return sqrtf(me->x * me->x + me->y * me->y + me->z * me->z + me->w * me->w);
}

float oquatf_get_dot(const quatf* me, const quatf* q)
{
	return me->x * q->x + me->y * q->y + me->z * q->z + me->w * q->w;
}

void oquatf_inverse(quatf* me)
{
	float dot = oquatf_get_dot(me, me);

	// conjugate
	for(int i = 0; i < 3; i++)
		me->arr[i] = -me->arr[i];
	
	for(int i = 0; i < 4; i++)
		me->arr[i] /= dot;
}

void oquatf_diff(const quatf* me, const quatf* q, quatf* out_q)
{
	quatf inv = *me;
	oquatf_inverse(&inv);
	oquatf_mult(&inv, q, out_q);
}

// filter queue

void ofq_init(filter_queue* me, int size)
{
	memset(me, 0, sizeof(filter_queue));
	me->size = size;
}

void ofq_add(filter_queue* me, const vec3f* vec)
{
	me->elems[me->at] = *vec;
    int x = me->at + 1;
    me->at = x % me->size;
}

void ofq_get_mean(const filter_queue* me, vec3f* vec)
{
	vec->x = vec->y = vec->z = 0;
	for(int i = 0; i < me->size; i++){
		vec->x += me->elems[i].x;
		vec->y += me->elems[i].y;
		vec->z += me->elems[i].z;
	}

	vec->x /= (float)me->size;
	vec->y /= (float)me->size;
	vec->z /= (float)me->size;
}
