/*
 * bmp180.h
 *
 *  Created on: 5 мар. 2020 г.
 *      Author: alexander
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f1xx_hal.h"

typedef struct BMP_180 {
	int16_t _AC1;
	int16_t _AC2;
	int16_t _AC3;
	uint16_t _AC4;
	uint16_t _AC5;
	uint16_t _AC6;
	int16_t _B1;
	int16_t _B2;
	int16_t _MB;
	int16_t _MC;
	int16_t _MD;

	int32_t Temperature;
	int32_t Pressure;


} Bmp180;

HAL_StatusTypeDef BMP_180_Init(Bmp180 *bmp, I2C_HandleTypeDef *i2c);

HAL_StatusTypeDef BMP_180_Update(Bmp180 *bmp, I2C_HandleTypeDef *i2c);
#endif /* INC_BMP180_H_ */
