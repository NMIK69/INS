#include "main.h"
#include "i2c.h"
#include "tim.h"

#include "unstuck_i2c.h"
#include "bitops.h"

static void unstuck_i2c1_rcc_reset(void);
static void unstuck_i2c1_toggle_scl(void);

/* delay in hunreds of nano secons */
/* for exampel delay_hns(10) would be a delay of 1000ns or 1us */
void delay_hns(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim16,0);
	while (__HAL_TIM_GET_COUNTER(&htim16) < us);
}

void unstuck_i2c1(void)
{
	/* see errata section 2.12.4 */
	if((hi2c1.ErrorCode & HAL_I2C_ERROR_BERR) != 0) {
		BIT_CLEAR(hi2c1.Instance->ISR, I2C_ISR_BERR_Pos);
	}
	else {
		unstuck_i2c1_toggle_scl();
		unstuck_i2c1_rcc_reset();
	}
}

static void unstuck_i2c1_rcc_reset(void)
{
	__HAL_RCC_I2C1_FORCE_RESET();
	__HAL_RCC_I2C1_RELEASE_RESET();

	HAL_I2C_Init(&hi2c1);
}

static void unstuck_i2c1_toggle_scl(void)
{
	/* 1. configure SCL Pin as GPIO output. */
	GPIO_InitTypeDef scl_gpio = {0};

	scl_gpio.Pin = I2C1_SCL_Pin;
	scl_gpio.Mode = GPIO_MODE_OUTPUT_PP;
	scl_gpio.Pull = GPIO_NOPULL;
	scl_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &scl_gpio);

	/* 2. toggle SCL 9 times. */
	uint8_t i = 0;
	while(i < 9) {
		HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET);
		delay_hns(10);

		HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);
		delay_hns(3);

		i += 1;
	}

	/* 3. set SCL Pin back to be used by the I2C peripheral. */
	scl_gpio.Pin = I2C1_SCL_Pin;
	scl_gpio.Mode = GPIO_MODE_AF_OD;
	scl_gpio.Pull = GPIO_NOPULL;
	scl_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	scl_gpio.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &scl_gpio);
}


