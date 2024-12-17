#ifndef INS_CAL_H
#define INS_CAL_H

#include "mpu6050.h"

void apply_accel_cal_params(struct mpu6050_measurement *mea);
void apply_gyro_cal_params(struct mpu6050_measurement *mea);

#endif //INS_CAL_H
