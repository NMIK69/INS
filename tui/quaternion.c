#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "quaternion.h"


struct quaternion quat_mul(struct quaternion q1, struct quaternion q2)
{
	struct quaternion res = {0};

	res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
	res.x = q1.x*q2.w + q1.w*q2.x - q1.z*q2.y + q1.y*q2.z;
	res.y = q1.y*q2.w + q1.z*q2.x + q1.w*q2.y - q1.x*q2.z;
	res.z = q1.z*q2.w - q1.y*q2.x + q1.x*q2.y + q1.w*q2.z;

	return res;
}

struct quaternion quat_norm(struct quaternion q)
{
	float w2 = q.w * q.w;
	float x2 = q.x * q.x;
	float y2 = q.y * q.y;
	float z2 = q.z * q.z;

	float n = sqrt((w2 + x2 + y2 + z2));
	assert(n != 0);

	struct quaternion res = {0};
	res.w = q.w / n;
	res.x = q.x / n;
	res.y = q.y / n;
	res.z = q.z / n;

	return res;
}

struct vec3f quat_to_euler(struct quaternion *q)
{
	struct vec3f rot = {0};

	float qw = q->w;
	float qx = q->x;
	float qy = q->y;
	float qz = q->z;

	float t0 = 2.0f * (qw * qx + qy * qz);
	float t1 = 1.0f - 2.0f * (qx * qx + qy *qy);
	rot.x = atan2(t0, t1);

	float t2 = 2.0f * (qw * qy - qz * qx);
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;
	rot.y = asin(t2);

	float t3 = 2.0f * (qw * qz + qx * qy);
	float t4 = 1.0f - 2.0f * (qy * qy + qz * qz);
	rot.z = atan2(t3, t4);


	rot.x = rot.x * 180.0f / M_PI;
	rot.y = rot.y * 180.0f / M_PI;
	rot.z = rot.z * 180.0f / M_PI;

	return rot;
}
