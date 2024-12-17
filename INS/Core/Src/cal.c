#include "cal.h"

static float M_gyro[3][3] = {
	      {-1.000460772433979, 0.0006898764366060253, -0.007252118626045198},
              {-0.0008338274248229802, -1.004183514242678, -0.013808156783547385},
              {0.016944045941235324, -0.018183474043750612, 1.0002381726925886}};

static float M_accel[3][3] = {
		{0.9943855091000198, 2.9808918530172742e-05, -0.0011450162924294836},
		{3.20642415919627e-05, 0.9994929778707495, -0.0003715308243724956},
		{-0.0011182699502468357, -0.00037135974559032503, 0.9864961817776355}};

static float b_accel[3] = {-0.010676239989301017, 0.005607300288838767, 0.08004243782803752};

void apply_accel_cal_params(struct mpu6050_measurement *mea)
{
	mea->accel.x = (mea->accel.x * M_accel[0][0]) + 
		       (mea->accel.y * M_accel[0][1]) +
		       (mea->accel.z * M_accel[0][2]) +
		       b_accel[0];

	mea->accel.y = (mea->accel.x * M_accel[1][0]) + 
		       (mea->accel.y * M_accel[1][1]) +
		       (mea->accel.z * M_accel[1][2]) +
		       b_accel[1];

	mea->accel.z = (mea->accel.x * M_accel[2][0]) + 
		       (mea->accel.y * M_accel[2][1]) +
		       (mea->accel.z * M_accel[2][2]) +
		       b_accel[2];
}

void apply_gyro_cal_params(struct mpu6050_measurement *mea)
{
	mea->gyro.x = (mea->gyro.x * M_gyro[0][0]) + 
		      (mea->gyro.y * M_gyro[0][1]) +
		      (mea->gyro.z * M_gyro[0][2]);

	mea->gyro.y = (mea->gyro.x * M_gyro[1][0]) + 
		      (mea->gyro.y * M_gyro[1][1]) +
		      (mea->gyro.z * M_gyro[1][2]);

	mea->gyro.z = (mea->gyro.x * M_gyro[2][0]) + 
		      (mea->gyro.y * M_gyro[2][1]) +
		      (mea->gyro.z * M_gyro[2][2]);
}
