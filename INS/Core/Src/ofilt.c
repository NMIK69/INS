#include <math.h>
#include "ofilt.h"

#define WIN_SIZE 5
#define WIN_MID (WIN_SIZE / 2)
#define NUM_DEV 10
#define MAGIC_VAL 1.4826f

#define SWAP(a, b, swp)\
	swp = a;\
	a = b;\
	b = swp;

#define SORT_THREE(arr, swp)\
	if (arr[0] > arr[1]) {\
		SWAP(arr[0],arr[1], swp)\
	}\
	if (arr[1] > arr[2]) {\
		SWAP(arr[1],arr[2], swp)\
	}\
	if (arr[0] > arr[1]) {\
		SWAP(arr[0],arr[1], swp)\
	}

static int of_ready = 0;
static struct mpu6050_measurement prev, curr, next;

int of_is_ready(void)
{
	return of_ready;
}

static float median_filter(float a, float b, float c)
{
	float tmp, median, mad;
	static float arr[3];
	static float adev[3];

	arr[0] = a;
	arr[1] = b;
	arr[2] = c;

	SORT_THREE(arr, tmp);
	median = arr[1];

	adev[0] = fabs(a - median);	
	adev[1] = fabs(b - median);	
	adev[2] = fabs(c - median);	

	SORT_THREE(adev, tmp);
	mad = adev[1];

	float diff = fabs(b - median);
	float cap = (NUM_DEV * mad * MAGIC_VAL);

	if(diff > cap)
		return median;
	return b;
}

void outlier_filter_init(struct mpu6050_measurement mea)
{
	static int cnt = 0;

	if(cnt == 0) {
		prev = mea;
	}

	if(cnt == 2) {
		curr = mea;
	}

	if(cnt == 4) {
		next = mea;
		of_ready = 1;
	}

	cnt += 1;
}

struct mpu6050_measurement outlier_filter_step(struct mpu6050_measurement mea)
{
	curr.accel.x = median_filter(prev.accel.x, curr.accel.x, next.accel.x);
	curr.accel.y = median_filter(prev.accel.y, curr.accel.y, next.accel.y);
	curr.accel.z = median_filter(prev.accel.z, curr.accel.z, next.accel.z);

	curr.gyro.x = median_filter(prev.gyro.x, curr.gyro.x, next.gyro.x);
	curr.gyro.y = median_filter(prev.gyro.y, curr.gyro.y, next.gyro.y);
	curr.gyro.z = median_filter(prev.gyro.z, curr.gyro.z, next.gyro.z);

	prev = curr;
	curr = next;
	next = mea;

	return prev;
}
