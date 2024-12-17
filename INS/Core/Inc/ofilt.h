#ifndef INS_OFILT_H
#define INS_OFILT_H

#include "mpu6050.h"


int of_is_ready(void);
void outlier_filter_init(struct mpu6050_measurement mea);
struct mpu6050_measurement outlier_filter_step(struct mpu6050_measurement mea);

#endif //INS_OFILT_H
