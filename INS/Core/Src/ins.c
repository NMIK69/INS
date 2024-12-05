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
	((sizeof(float) * 6) +\
	 (sizeof(float) * 4) +\
	 (sizeof(uint32_t)) +\
	 (sizeof(uint8_t)))\

static uint8_t transmit_buf[TRANSMIT_DATA_SIZE] = {MPU6050_MEA_START};

static uint8_t mea_buf[40][MPU6050_RAW_DATA_SIZE];
static size_t mea_end = 0;
static uint32_t mea_ts = 0;

static int mpu6050_ready = 0;
static int ins_ready = 0;
static int of_ready = 0;

static float aref_x = 0.0f;
static float aref_y = 0.0f;
static float aref_z = 0.0f;

static float gbias_x = 0.0f;
static float gbias_y = 0.0f;
static float gbias_z = 0.0f;

static float M_accel[3][3] = {
		{0.9943855091000198, 2.9808918530172742e-05, -0.0011450162924294836},
		{3.20642415919627e-05, 0.9994929778707495, -0.0003715308243724956},
		{-0.0011182699502468357, -0.00037135974559032503, 0.9864961817776355}
				};

static float b_accel[3] = {-0.010676239989301017, 0.005607300288838767, 0.08004243782803752};

static void apply_cal_params(struct mpu6050_measurement *mea)
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

static struct kalman_filter *kf;

static int kf_i = 0;

int _write(int file, char *ptr, int len) {
	HAL_StatusTypeDef res = HAL_UART_Transmit(&huart2, (uint8_t*)ptr,
							len, 100);
	assert(res == HAL_OK);
	ptr += len;
	return len;
}

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

static struct mpu6050_measurement prev, curr, next;
static void outlier_filter_init(struct mpu6050_measurement mea)
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

static void outlier_filter(void)
{
	curr.accel.x = median_filter(prev.accel.x, curr.accel.x, next.accel.x);
	curr.accel.y = median_filter(prev.accel.y, curr.accel.y, next.accel.y);
	curr.accel.z = median_filter(prev.accel.z, curr.accel.z, next.accel.z);

	curr.gyro.x = median_filter(prev.gyro.x, curr.gyro.x, next.gyro.x);
	curr.gyro.y = median_filter(prev.gyro.y, curr.gyro.y, next.gyro.y);
	curr.gyro.z = median_filter(prev.gyro.z, curr.gyro.z, next.gyro.z);
	
}
static void outlier_filter_advance(struct mpu6050_measurement mea)
{
	prev = curr;
	curr = next;
	next = mea;
}

static void ins_dyn_init(struct mpu6050_measurement *mea)
{
	static int cnt = 0;

	/* obtain avg of accel (for kf) */
	/* obtain avg of ang vel (for gyro bias) */
	if(cnt < 5000) {

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
	}
}

static void transmit_measurements(struct quaternion q, struct mpu6050_measurement mea)
{
	/* copy data read from mpu6050 to transmit buffer */
	float m_arr[] = {mea.accel.x, mea.accel.y, mea.accel.z,
			 mea.gyro.x, mea.gyro.y, mea.gyro.z};
	memcpy(&transmit_buf[1], m_arr, sizeof(m_arr));

	/* copy estimated orientation as quaternion into transmit buffer after
	 * raw data. */
	size_t q_pos = 1 + sizeof(m_arr);
	float q_arr[] = {q.w, q.x, q.y, q.z};
	//struct quaternion qt = {1.23, 4.56, 7.89, 10.1112};
	//float q_arr[] = {qt.w, qt.x, qt.y, qt.z};
	memcpy(&transmit_buf[q_pos], q_arr, sizeof(q_arr));

	/* copy timestamp to transmit buffer after orientation quaternion. */
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
		mea_ts = HAL_GetTick();
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
	//float var_a = 0.01*0.01;
	//float var_w = 0.003*0.003;
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
	while(1) {
		if(i != mea_end) {
			HAL_GPIO_WritePin(KF_Timing_GPIO_Port, KF_Timing_Pin, GPIO_PIN_SET);
			
			assert(i < ARR_SIZE(mea_buf));

			struct mpu6050_measurement mea = mpu6050_decode_raw(mea_buf[i]);
			apply_cal_params(&mea);

			//get_minmax2(&mea);

			if(of_ready == 0) {
				outlier_filter_init(mea);
			}

			else if(ins_ready == 0) {
				outlier_filter();

				ins_dyn_init(&curr);

				outlier_filter_advance(mea);
			}
			else {
				outlier_filter();

				int err = kf_filt(kf, 
					-curr.gyro.x, -curr.gyro.y, curr.gyro.z,
					curr.accel.x, curr.accel.y, curr.accel.z);

				assert(err == 0);

				tx_rate_cnt += 1;
				if(tx_rate_cnt == 10) {
					transmit_measurements(kf->q, curr);
					tx_rate_cnt = 0;
				}

				outlier_filter_advance(mea);
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
