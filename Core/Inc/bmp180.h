/*
 * bmp180.h
 *
 *  Created on: 5 мар. 2020 г.
 *      Author: alexander
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f1xx_hal.h"

typedef enum BMP_STATUS{
	Bmp_Initialization_Error = 0x00,
	Bmp_Data_Transmit_Error,
	Bmp_Timeout,
	Bmp_Ok
} BmpStatus;

typedef struct BMP_180 {
	uint8_t isInit;
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

	int32_t _t_last_1;
	int32_t _p_last_1;

	int32_t _t_last_2;
	int32_t _p_last_2;

	int32_t _t_last_3;
	int32_t _p_last_3;

	I2C_HandleTypeDef *_i2c;

	int32_t Temperature;
	int32_t Pressure;

} Bmp180;

BmpStatus BMP_180_Init(Bmp180 *bmp, I2C_HandleTypeDef *i2c);

BmpStatus BMP_180_Update(Bmp180 *bmp);

#endif /* INC_BMP180_H_ */
