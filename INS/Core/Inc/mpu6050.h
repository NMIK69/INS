#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

#define MPU6050_I2C_ADDR (0x68 << 1)
#define MPU6050_WHO_AM_I_VAL 0x68
#define MPU6050_HREG 0x75
#define MPU6050_LREG 0x0d
#define MPU6050_WHOAIMI_REG 0x75
#define MPU6050_PWR_MGMT1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_CONFIG 0x1A
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_DATA_BASE 0x3B

#define MPU6050_SELF_TEST_X 0x0D
#define MPU6050_SELF_TEST_Y 0x0E
#define MPU6050_SELF_TEST_Z 0x0F
#define MPU6050_SELF_TEST_A 0x10

/* 3 axis accel + 3 axis gyro + temperature. Each measurement (for each axis
 * and temperature) is 16 bit or 2 bytes. */
#define MPU6050_RAW_DATA_SIZE (2*3 + 2*3 + 2*1)

struct vec3f
{
	float x, y, z;
};

struct mpu6050_measurement
{
	struct vec3f accel;
	struct vec3f gyro;
};

struct mpu6050_config
{
	I2C_HandleTypeDef *hi2c;

	/* digital low pass filter config (stage) */
	uint8_t dplf_cfg;

	enum mpu6050_samplerates
	{
		MPU6050_SMPLRT_10HZ = 10U,
		MPU6050_SMPLRT_50HZ = 50U,
		MPU6050_SMPLRT_100HZ = 100U,
		MPU6050_SMPLRT_150HZ = 150U,
		MPU6050_SMPLRT_200HZ = 200U,
		MPU6050_SMPLRT_500HZ = 500U,
		MPU6050_SMPLRT_800HZ = 800U,
		MPU6050_SMPLRT_1000HZ = 1000U,

	} smplrt;

	enum mpu6050_gyro_fs_ranges
	{
		MPU6050_GYRO_FSR_250DPS = 0U,
		MPU6050_GYRO_FSR_500DPS = 1U,
		MPU6050_GYRO_FSR_1000DPS = 2U,
		MPU6050_GYRO_FSR_2000DPS = 3U,
	} gyro_fs_sel;

	enum mpu6050_accel_fs_ranges
	{
		MPU6050_ACCEL_FSR_2G = 0U,
		MPU6050_ACCEL_FSR_4G = 1U,
		MPU6050_ACCEL_FSR_8G = 2U,
		MPU6050_ACCEL_FSR_16G = 3U,
	} accel_fs_sel;

	/* enable interrupt latching and interrupt clearing by reading any
	 * register? */
	int latch_int_rd_clear_en;
};

void mpu6050_configure(struct mpu6050_config *conf);
struct mpu6050_measurement mpu6050_decode_raw(const uint8_t *raw_data);
void mpu6050_set_gyro_bias(float x, float y, float z);
HAL_StatusTypeDef mpu6050_read_it(
		uint16_t base_reg, uint16_t nregs, 
		uint8_t *rec_data);

#endif //MPU6050_H
