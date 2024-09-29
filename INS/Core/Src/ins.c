#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "main.h"

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>

#include "ins.h"
#include "kalman.h"
#include "quaternion.h"
#include "mpu6050.h"
#include "bitops.h"
#include "unstuck_i2c.h"

#define ARR_SIZE(arr) (sizeof(arr) / sizeof(*arr))

#define MPU6050_MEA_START 0xAA
/* transmit all raw measurement values + orientation as quaternion (4 floats)
   + timestamp (one uint32_t) + measurement start identifier byte. */
#define TRANSMIT_DATA_SIZE\
	((MPU6050_RAW_DATA_SIZE) +\
	 (sizeof(float) * 4) +\
	 (sizeof(uint32_t)) +\
	 (sizeof(uint8_t)))\

static uint8_t transmit_buf[TRANSMIT_DATA_SIZE] = {MPU6050_MEA_START};

static uint8_t mea_buf[40][MPU6050_RAW_DATA_SIZE];
static size_t mea_end = 0;
static uint32_t mea_ts = 0;

static int mpu6050_ready = 0;

static float aref_x = 0.0f;
static float aref_y = 0.0f;
static float aref_z = 0.0f;

static struct kalman_filter *kf;
static int kf_ready = 0;
static int kf_ref_count = 0;

static int kf_i = 0;

int _write(int file, char *ptr, int len) {
	HAL_StatusTypeDef res = HAL_UART_Transmit(&huart2, (uint8_t*)ptr,
							len, 100);
	assert(res == HAL_OK);
	ptr += len;
	return len;
}

static void make_kf_a_ref(struct mpu6050_measurement *mea)
{
	if(kf_ref_count < 20) {
		aref_x += mea->accel.x;
		aref_y += mea->accel.y;
		aref_z += mea->accel.z;
		kf_ref_count += 1;
	}
	else {
		kf_ready = 1;
		kf_set_aref(kf, aref_x, aref_y, aref_z);
	}
}

static void transmit_measurements(struct quaternion q, const uint8_t *raw_data)
{
	/* copy raw data read from mpu6050 to transmit buffer */
	memcpy(&transmit_buf[1], raw_data, MPU6050_RAW_DATA_SIZE);

	/* copy estimated orientation as quaternion into transmit buffer after
	 * raw data. */
	size_t q_pos = 1 + MPU6050_RAW_DATA_SIZE;
	float q_arr[sizeof(q.w) * 4] = {q.w, q.x, q.y, q.z};
	memcpy(&transmit_buf[q_pos], q_arr, ARR_SIZE(q_arr));

	/* copy timestamp to transmit buffer after orientation quaternion. */
	/* TODO: use rtc to make timestamps. */
	size_t ts_pos = q_pos + ARR_SIZE(q_arr);
	memcpy(&transmit_buf[ts_pos], &mea_ts, sizeof(mea_ts));


	size_t len = ts_pos + sizeof(mea_ts);
	assert(len == TRANSMIT_DATA_SIZE);


	HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(
						&huart2,
						transmit_buf,
						TRANSMIT_DATA_SIZE);

	assert(ret == HAL_OK);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == MPU6050_INT_PIN_Pin && mpu6050_ready == 1) {
		static size_t i = 0;

		assert(i < ARR_SIZE(mea_buf));

		HAL_GPIO_TogglePin(I2C_Timing_GPIO_Port, I2C_Timing_Pin);

		HAL_StatusTypeDef ret = mpu6050_read_it(MPU6050_DATA_BASE, 14, mea_buf[i]);


		/* When HAL_ERROR is returned, it should have been handled by
		 * the i2c error callback. */
		if(ret == HAL_BUSY) {
			HAL_GPIO_WritePin(I2C_ERR_INDI_GPIO_Port, I2C_ERR_INDI_Pin, GPIO_PIN_SET);
			unstuck_i2c1();
		}
		/* ringbuffer */
		else {
			i += 1;
			if(i == ARR_SIZE(mea_buf))
				i = 0;
		}
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* ringbuffer */
	if(hi2c->Instance == hi2c1.Instance) {
		HAL_GPIO_TogglePin(I2C_Timing_GPIO_Port, I2C_Timing_Pin);
		mea_end += 1;

		if(mea_end == ARR_SIZE(mea_buf))
			mea_end = 0;

		assert(mea_end != kf_i);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == hi2c1.Instance) {
		HAL_GPIO_WritePin(I2C_ERR_INDI_GPIO_Port, I2C_ERR_INDI_Pin, GPIO_PIN_SET);
		unstuck_i2c1();
	}
}


static void ins_init(void)
{
	HAL_Delay(1000);

	struct mpu6050_config conf = {0};
	conf.hi2c = &hi2c1; 
	conf.dplf_cfg = 2;
	conf.smplrt = MPU6050_SMPLRT_500HZ;
	conf.accel_fs_sel = MPU6050_ACCEL_FSR_4G;
	conf.gyro_fs_sel = MPU6050_GYRO_FSR_2000DPS;
	conf.latch_int_rd_clear_en = 0;
	
	assert(sizeof(float) == sizeof(uint32_t));
	
	mpu6050_configure(&conf);
	
	float dt = 1.0f/(float)conf.smplrt;
	float var_a = 0.01*0.01;
	float var_w = 0.003*0.003;
	float var_P = 0.000001;
	kf = kf_init(dt, var_a, var_w, var_P);
	/* state quaternion is identity quaternion by default. */
	
	mpu6050_ready = 1;
}

void ins_run(void)
{
	ins_init();

	size_t i = 0;
	size_t tx_rate_cnt = 0;
	while(1) {
		if(i != mea_end) {
			HAL_GPIO_WritePin(KF_Timing_GPIO_Port, KF_Timing_Pin, GPIO_PIN_SET);
			
			assert(i < ARR_SIZE(mea_buf));

			struct mpu6050_measurement mea = mpu6050_decode_raw(mea_buf[i]);

			if(kf_ready == 0) {
				make_kf_a_ref(&mea);
			}
			else {
				
				int err = kf_filt(kf, 
					mea.gyro.x, mea.gyro.y, mea.gyro.z,
					mea.accel.x, mea.accel.y, mea.accel.z);

				assert(err == 0);

				tx_rate_cnt += 1;
				if(tx_rate_cnt == 100) {
					transmit_measurements(kf->q, mea_buf[i]);
					tx_rate_cnt = 0;
				}
			}

			/* ringbuffer */
			i += 1;
			if(i > mea_end && i == ARR_SIZE(mea_buf))
				i = 0;

			kf_i = i;

			HAL_GPIO_WritePin(KF_Timing_GPIO_Port, KF_Timing_Pin, GPIO_PIN_RESET);
		}
	}
}
