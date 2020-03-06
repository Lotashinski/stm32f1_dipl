/*
 * tca9548a.c
 *
 *  Created on: Mar 6, 2020
 *      Author: alexander
 */

#include "tca9548a.h"

#define TCA_ADDRESS 0xE0
#define TCA_CONTROL 0x3B
#define I2C_TIMEOUT 1000

HAL_StatusTypeDef TCA_Set(I2C_HandleTypeDef *i2c,
		TcaAddress address) {

	uint8_t data[2] = {TCA_CONTROL, 1 << address};

	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(i2c, TCA_ADDRESS, data, 2, I2C_TIMEOUT);
	if (res != HAL_OK)
		return res;

	return res;
}
