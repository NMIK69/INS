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
#include "ofilt.h"
#include "cal.h"

#define ARR_SIZE(arr) (sizeof(arr) / sizeof(*arr))
#define INIT_SMPLS_CNT 5000

//#define LA_DEBUG_EN

#define MPU6050_MEA_START 0xAA
/* transmit all raw measurement values + orientation as quaternion (4 floats)
   + timestamp (one uint32_t) + measurement start identifier byte. */
#define TRANSMIT_DATA_SIZE\
	((sizeof(float) * 6) +\
	 (sizeof(float) * 4) +\
	 (sizeof(uint32_t)) +\
	 (sizeof(uint8_t)))\

static uint8_t transmit_buf[TRANSMIT_DATA_SIZE] = {MPU6050_MEA_START};

/* ringbuffer */
static uint8_t mea_buf[40][MPU6050_RAW_DATA_SIZE];
static size_t mea_end = 0;

static uint32_t mea_ts = 0;

static int mpu6050_ready = 0;
static int ins_ready = 0;

static float aref_x = 0.0f;
static float aref_y = 0.0f;
static float aref_z = 0.0f;

static float gbias_x = 0.0f;
static float gbias_y = 0.0f;
static float gbias_z = 0.0f;

static struct kalman_filter *kf;

/* for ringbuffer overrun check */
static int consumer_idx = 0;

static void ins_dyn_init(struct mpu6050_measurement *mea)
{
	static int cnt = 0;

	/* obtain avg of accel (for kf) */
	/* obtain avg of ang vel (for gyro bias) */
	if(cnt < INIT_SMPLS_CNT) {

		aref_x += mea->accel.x;
		aref_y += mea->accel.y;
		aref_z += mea->accel.z;

		gbias_x += mea->gyro.x;
		gbias_y += mea->gyro.y;
		gbias_z += mea->gyro.z;

		cnt += 1;
	}
	else {
		aref_x = aref_x / cnt;
		aref_y = aref_y / cnt;
		aref_z = aref_z / cnt;

		gbias_x = -gbias_x / cnt;
		gbias_y = -gbias_y / cnt;
		gbias_z = -gbias_z / cnt;

		kf_set_aref(kf, aref_x, aref_y, aref_z);
		mpu6050_set_gyro_bias(gbias_x, gbias_y, gbias_z);

		cnt = 0;
		ins_ready = 1;
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
	}
}

static void transmit_measurements(struct quaternion q, struct mpu6050_measurement mea)
{
	float m_arr[] = {mea.accel.x, mea.accel.y, mea.accel.z,
			 mea.gyro.x, mea.gyro.y, mea.gyro.z};
	memcpy(&transmit_buf[1], m_arr, sizeof(m_arr));

	size_t q_pos = 1 + sizeof(m_arr);
	float q_arr[] = {q.w, q.x, q.y, q.z};
	memcpy(&transmit_buf[q_pos], q_arr, sizeof(q_arr));

	size_t ts_pos = q_pos + sizeof(q_arr);
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

		#ifdef LA_DEBUG_EN
		HAL_GPIO_TogglePin(I2C_Timing_GPIO_Port, I2C_Timing_Pin);
		#endif

		HAL_StatusTypeDef ret = mpu6050_read_it(MPU6050_DATA_BASE, 14, mea_buf[i]);


		/* When HAL_ERROR is returned, it should have been handled by
		 * the i2c error callback. */
		if(ret == HAL_BUSY) {
			#ifdef LA_DEBUG_EN
			HAL_GPIO_WritePin(I2C_ERR_INDI_GPIO_Port, 
					I2C_ERR_INDI_Pin, GPIO_PIN_SET);
			#endif

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
		#ifdef LA_DEBUG_EN
		HAL_GPIO_TogglePin(I2C_Timing_GPIO_Port, I2C_Timing_Pin);
		#endif

		mea_ts = HAL_GetTick();
		mea_end += 1;

		if(mea_end == ARR_SIZE(mea_buf))
			mea_end = 0;

		assert(mea_end != consumer_idx);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == hi2c1.Instance) {
		#ifdef LA_DEBUG_EN
		HAL_GPIO_WritePin(I2C_ERR_INDI_GPIO_Port, I2C_ERR_INDI_Pin, GPIO_PIN_SET);
		#endif

		unstuck_i2c1();
	}
}


static void ins_setup(void)
{
	HAL_Delay(1000);

	struct mpu6050_config conf = {0};
	conf.hi2c = &hi2c1; 
	conf.dplf_cfg = 4;
	conf.smplrt = MPU6050_SMPLRT_500HZ;
	conf.accel_fs_sel = MPU6050_ACCEL_FSR_4G;
	conf.gyro_fs_sel = MPU6050_GYRO_FSR_2000DPS;
	conf.latch_int_rd_clear_en = 0;
	
	assert(sizeof(float) == sizeof(uint32_t));
	
	mpu6050_configure(&conf);
	
	float dt = 1.0f/(float)conf.smplrt;
	float var_a = 0.005*0.005;
	float var_w = 0.001*0.001;
	float var_P = 0.000001;
	kf = kf_init(dt, var_a, var_w, var_P);
	/* state quaternion is identity quaternion by default. */
	
	mpu6050_ready = 1;
}

void ins_run(void)
{
	ins_setup();

	size_t i = 0;
	size_t tx_rate_cnt = 0;

	HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

	while(1) {
		if(i != mea_end) {
			#ifdef LA_DEBUG_EN
			HAL_GPIO_WritePin(KF_Timing_GPIO_Port, KF_Timing_Pin, GPIO_PIN_SET);
			#endif
			
			assert(i < ARR_SIZE(mea_buf));

			struct mpu6050_measurement mea = mpu6050_decode_raw(mea_buf[i]);
			apply_accel_cal_params(&mea);


			if(of_is_ready() == 0) {
				outlier_filter_init(mea);
			}

			else if(ins_ready == 0) {
				mea = outlier_filter_step(mea);

				ins_dyn_init(&mea);
			}
			else {
				apply_gyro_cal_params(&mea);

				mea = outlier_filter_step(mea);

				int err = kf_filt(kf, 
					mea.gyro.x, mea.gyro.y, mea.gyro.z,
					mea.accel.x, mea.accel.y, mea.accel.z);

				assert(err == 0);

				tx_rate_cnt += 1;
				if(tx_rate_cnt == 2) {
					transmit_measurements(kf->q, mea);
					tx_rate_cnt = 0;
				}
			}


			/* ringbuffer */
			i += 1;
			if(i > mea_end && i == ARR_SIZE(mea_buf))
				i = 0;

			consumer_idx = i;

			#ifdef LA_DEBUG_EN
			HAL_GPIO_WritePin(KF_Timing_GPIO_Port, KF_Timing_Pin, GPIO_PIN_RESET);
			#endif
		}
	}
}
