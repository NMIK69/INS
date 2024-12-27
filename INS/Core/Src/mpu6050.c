#include <assert.h>

#include "mpu6050.h"
#include "math.h"
#include "bitops.h"
#include "unstuck_i2c.h"
#include "i2c.h"

static float mpu6050_gyro_lsbs;
static float mpu6050_accel_lsbs;

static float gbias_x = 0.0f;
static float gbias_y = 0.0f;
static float gbias_z = 0.0f;

static I2C_HandleTypeDef *mpu6050_hi2c;

inline static int is_valid_mpu6050_reg(uint16_t reg);
static void mpu6050_write_reg(uint16_t reg, uint8_t val);
static uint8_t mpu6050_read_reg(uint16_t reg);
static void mpu6050_test_whoami(void);
static void mpu6050_disable_sleep(void);
static void mpu6050_reset_device(void);
static void mpu6050_reset_sensor_paths(void);
static void mpu6050_clksel_xgryo(void);
static void mpu6050_set_smplrt_div(uint8_t smplrt_div);
static void mpu6050_set_fsr_gyro(enum mpu6050_gyro_fs_ranges fs);
static void mpu6050_set_fsr_accel(enum mpu6050_accel_fs_ranges fs);
static void mpu6050_set_dplf(uint8_t dplf_sel);
static void mpu6050_data_ready_interrupt_enable(void);
static void mpu6050_enable_latch_int_rd_clear(void);


HAL_StatusTypeDef mpu6050_read_it(
		uint16_t base_reg, uint16_t nregs, 
		uint8_t *rec_data)
{
	assert(is_valid_mpu6050_reg(base_reg));
	assert(is_valid_mpu6050_reg(base_reg + nregs - 1));

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_IT(
					mpu6050_hi2c,
					MPU6050_I2C_ADDR,
					base_reg,
					I2C_MEMADD_SIZE_8BIT,
					rec_data,
					nregs);
	return ret;
}

void mpu6050_configure(struct mpu6050_config *conf)
{
	/* in Hz */
	unsigned int gyro_out_rate = 1000;

	assert(conf->hi2c != NULL);
	mpu6050_hi2c = conf->hi2c;

	mpu6050_test_whoami();

	/* reset device */
	mpu6050_reset_device();

	/* reset sensor paths. */
	mpu6050_reset_sensor_paths();

	/* disable sleep mode */
	mpu6050_disable_sleep();

	/* change clk source */
	mpu6050_clksel_xgryo();

	/* determine smplrt_div */
	if(conf->dplf_cfg == 0 || conf->dplf_cfg == 7)
		gyro_out_rate = 8000;

	assert((gyro_out_rate % conf->smplrt) == 0);
	unsigned int smplrt_div	= (gyro_out_rate / conf->smplrt) - 1;
	assert(smplrt_div <= 0xff);

	/* set smplrt_div */
	mpu6050_set_smplrt_div((uint8_t)smplrt_div);

	/* set dplf config */
	mpu6050_set_dplf(conf->dplf_cfg);

	/* set gyro fs range */
	mpu6050_set_fsr_gyro(conf->gyro_fs_sel);

	/* set accel fs range */
	mpu6050_set_fsr_accel(conf->accel_fs_sel);

	/* enable interrupt on data ready */
	mpu6050_data_ready_interrupt_enable();

	/* enable int latching and rd clear? */
	if(conf->latch_int_rd_clear_en == 1)
		mpu6050_enable_latch_int_rd_clear();
}

void mpu6050_set_gyro_bias(float x, float y, float z)
{
	gbias_x = x;
	gbias_y = y;
	gbias_z = z;
}

struct mpu6050_measurement mpu6050_decode_raw(const uint8_t *raw_data)
{
	struct mpu6050_measurement mea = {0};

	mea.accel.x = (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]) / mpu6050_accel_lsbs;
	mea.accel.y = (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]) / mpu6050_accel_lsbs;
	mea.accel.z = (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]) / mpu6050_accel_lsbs;

	mea.gyro.x = (int16_t)(((int16_t)raw_data[8] << 8)  | raw_data[9]) / mpu6050_gyro_lsbs;
	mea.gyro.y = (int16_t)(((int16_t)raw_data[10] << 8) | raw_data[11]) / mpu6050_gyro_lsbs;
	mea.gyro.z = (int16_t)(((int16_t)raw_data[12] << 8) | raw_data[13]) / mpu6050_gyro_lsbs;

	/* conversion from deg/s to rad/s */
	mea.gyro.x = mea.gyro.x * M_PI / 180.0f;
	mea.gyro.y = mea.gyro.y * M_PI / 180.0f;
	mea.gyro.z = mea.gyro.z * M_PI / 180.0f;

	mea.gyro.x += gbias_x;
	mea.gyro.y += gbias_y;
	mea.gyro.z += gbias_z;

	return mea;
}


static void mpu6050_write_reg(uint16_t reg, uint8_t val)
{
	assert(is_valid_mpu6050_reg(reg));

	HAL_StatusTypeDef res = HAL_I2C_Mem_Write(
					mpu6050_hi2c,
					MPU6050_I2C_ADDR,
					reg,
					I2C_MEMADD_SIZE_8BIT,
					&val,
					1,
					1);
	assert(res == HAL_OK);
}

static uint8_t mpu6050_read_reg(uint16_t reg)
{
	assert(is_valid_mpu6050_reg(reg));

	uint8_t reg_val;

	HAL_StatusTypeDef res = HAL_I2C_Mem_Read(
					mpu6050_hi2c,
					MPU6050_I2C_ADDR,
					reg,
					I2C_MEMADD_SIZE_8BIT,
					&reg_val,
					1,
					1);

	if(res != HAL_OK) {

		#ifdef LA_DEBUG_EN
		HAL_GPIO_WritePin(I2C_ERR_INDI_GPIO_Port, I2C_ERR_INDI_Pin, GPIO_PIN_SET);
		#endif

		unstuck_i2c1();
		res = HAL_I2C_Mem_Read(
				mpu6050_hi2c,
				MPU6050_I2C_ADDR,
				reg,
				I2C_MEMADD_SIZE_8BIT,
				&reg_val,
				1,
				1);
	}

	assert(res == HAL_OK);

	return reg_val;
}

static void mpu6050_test_whoami(void)
{
	uint8_t wai = mpu6050_read_reg(MPU6050_WHOAIMI_REG);

	assert(wai == MPU6050_WHO_AM_I_VAL);
}

static void mpu6050_disable_sleep(void)
{
	uint8_t reg_val = mpu6050_read_reg(MPU6050_PWR_MGMT1);

	/* clear sleep bit */
	BIT_CLEAR(reg_val, 6);

	mpu6050_write_reg(MPU6050_PWR_MGMT1, reg_val);
}

static void mpu6050_reset_device(void)
{
	uint8_t reg_val = 1<<7;
	mpu6050_write_reg(MPU6050_PWR_MGMT1, reg_val);

	/* reference manual tells me to wait at least 100 ms. */
	HAL_Delay(200);

	/* once bit 7 is cleared, device reset is done. */
	/* 0x40 is reset value of that register. */
	while(reg_val != 0x40) {
		reg_val = mpu6050_read_reg(MPU6050_PWR_MGMT1);
	}
}

static void mpu6050_reset_sensor_paths(void)
{
	uint8_t reg_val = 0x07;
	mpu6050_write_reg(MPU6050_SIGNAL_PATH_RESET, reg_val);

	/* just my autism and paranoia*/
	HAL_Delay(200);
}

static void mpu6050_clksel_xgryo(void)
{
	/* Changing the clk to one of the gyroscope clocks this is advertised in the
	 * register description document as recomended. See: 4.28 */

	/* change clk to x axis gyro refernce */
	uint8_t reg_val = mpu6050_read_reg(MPU6050_PWR_MGMT1);

	BIT_SET(reg_val, 0);
	mpu6050_write_reg(MPU6050_PWR_MGMT1, reg_val);
}

static void mpu6050_set_smplrt_div(uint8_t smplrt_div)
{
	mpu6050_write_reg(MPU6050_SMPLRT_DIV, smplrt_div);
}

static void mpu6050_set_fsr_gyro(enum mpu6050_gyro_fs_ranges fs)
{
	uint8_t reg_val = mpu6050_read_reg(MPU6050_GYRO_CONFIG);

	BITMASK_CLEAR_AND_SET(reg_val, (0x3<<3), (fs << 3));

	mpu6050_write_reg(MPU6050_GYRO_CONFIG, reg_val);

	switch(fs) {
	case MPU6050_GYRO_FSR_250DPS:
		mpu6050_gyro_lsbs = 131.0f;
		break;
	case MPU6050_GYRO_FSR_500DPS:
		mpu6050_gyro_lsbs = 65.5f;
		break;
	case MPU6050_GYRO_FSR_1000DPS:
		mpu6050_gyro_lsbs = 32.8f;
		break;
	case MPU6050_GYRO_FSR_2000DPS:
		mpu6050_gyro_lsbs = 16.4f;
		break;
	default:
		assert(0);
	}
}

static void mpu6050_set_fsr_accel(enum mpu6050_accel_fs_ranges fs)
{
	uint8_t reg_val = mpu6050_read_reg(MPU6050_ACCEL_CONFIG);

	BITMASK_CLEAR_AND_SET(reg_val, (0x3<<3), (fs << 3));

	mpu6050_write_reg(MPU6050_ACCEL_CONFIG, reg_val);

	switch(fs) {
	case MPU6050_ACCEL_FSR_2G:
		mpu6050_accel_lsbs = 16384.0f;
		break;
	case MPU6050_ACCEL_FSR_4G:
		mpu6050_accel_lsbs = 8192.0f;
		break;
	case MPU6050_ACCEL_FSR_8G:
		mpu6050_accel_lsbs = 4096.0f;
		break;
	case MPU6050_ACCEL_FSR_16G:
		mpu6050_accel_lsbs = 2048.0f;
		break;
	default:
		assert(0);
	}
}

static void mpu6050_set_dplf(uint8_t dplf_sel)
{
	uint8_t reg_val = mpu6050_read_reg(MPU6050_CONFIG);

	BITMASK_CLEAR_AND_SET(reg_val, 0x7, dplf_sel);

	mpu6050_write_reg(MPU6050_CONFIG, reg_val);
}

static void mpu6050_data_ready_interrupt_enable(void)
{
	uint8_t reg_val = mpu6050_read_reg(MPU6050_INT_ENABLE);

	BIT_SET(reg_val, 0);
	mpu6050_write_reg(MPU6050_INT_ENABLE, reg_val);
}

static void mpu6050_enable_latch_int_rd_clear(void)
{
	uint8_t reg_val = mpu6050_read_reg(MPU6050_INT_PIN_CFG);

	BIT_SET(reg_val, 5);
	BIT_SET(reg_val, 4);

	mpu6050_write_reg(MPU6050_INT_PIN_CFG, reg_val);
}

inline static int is_valid_mpu6050_reg(uint16_t reg)
{
	return (reg >= MPU6050_LREG && reg <= MPU6050_HREG);
}

