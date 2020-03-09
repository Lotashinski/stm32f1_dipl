/*
 * bmp180.c
 *
 *  Created on: 5 мар. 2020 г.
 *      Author: alexander
 */

#include "bmp180.h"

#define BMP_ADDRESS                 0xEE

#define I2C_TIMEOUT                 1000

#define BMP_AC1_H                   0xAA
#define BMP_CONTROL                 0xF4
#define BMP_MSB                     0xF6

#define BMP_MODE_TEMPERATURE        0x2E
#define BMP_TEMPERATURE_DELAY      	5
#define BMP_MODE_PRESSURE           0xF4
#define BMP_OSS                     8
#define BMP_PRESSURE_DELAY          26

BmpStatus _update(Bmp180 *bmp, I2C_HandleTypeDef *i2c);

BmpStatus BMP_180_Init(Bmp180 *bmp, I2C_HandleTypeDef *i2c) {
	bmp->isInit = 0;
	HAL_StatusTypeDef res;
	uint8_t reg = BMP_AC1_H;

	bmp->_i2c = i2c;

	res = HAL_I2C_Master_Transmit(i2c, BMP_ADDRESS, &reg, 1,
	I2C_TIMEOUT);

	if (res != HAL_OK)
		return Bmp_Initialization_Error;

	uint8_t data[22] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0 };

	res = HAL_I2C_Master_Receive(i2c, BMP_ADDRESS, data, 22, I2C_TIMEOUT);

	if (res != HAL_OK)
		return Bmp_Data_Transmit_Error;

	bmp->isInit = 1;

	bmp->_AC1 = ((int16_t) data[0] << 8) | data[1];
	bmp->_AC2 = ((int16_t) data[2] << 8) | data[3];
	bmp->_AC3 = ((int16_t) data[4] << 8) | data[5];
	bmp->_AC4 = ((uint16_t) data[6] << 8) | data[7];
	bmp->_AC5 = ((uint16_t) data[8] << 8) | data[9];
	bmp->_AC6 = ((uint16_t) data[10] << 8) | data[11];
	bmp->_B1 = ((int16_t) data[12] << 8) | data[13];
	bmp->_B2 = ((int16_t) data[14] << 8) | data[15];
	bmp->_MB = ((int16_t) data[16] << 8) | data[17];
	bmp->_MC = ((int16_t) data[18] << 8) | data[19];
	bmp->_MD = ((int16_t) data[20] << 8) | data[21];

	_update(bmp, i2c);
	bmp->_p_last_1 = bmp->Pressure;
	bmp->_t_last_1 = bmp->Temperature;

	_update(bmp, i2c);
	bmp->_p_last_2 = bmp->_p_last_1;
	bmp->_t_last_2 = bmp->_t_last_1;
	bmp->_p_last_1 = bmp->Pressure;
	bmp->_t_last_1 = bmp->Temperature;

	BMP_180_Update(bmp);

	return Bmp_Ok;
}

BmpStatus _update(Bmp180 *bmp, I2C_HandleTypeDef *i2c) {
	HAL_StatusTypeDef res;

	/*
	 * read temperature
	 */
	uint8_t to_transmit[2] = { BMP_CONTROL, BMP_MODE_TEMPERATURE };
	uint8_t receive_data[5] = { 0, 0, 0, 0, 0 };

	uint8_t *msc = &receive_data[0];
	uint8_t *msb = &receive_data[2];
	uint8_t *lsb = &receive_data[3];
	uint8_t *xlsb = &receive_data[4];

	// start the measurement
	res = HAL_I2C_Master_Transmit(i2c, BMP_ADDRESS, to_transmit, 2,
	I2C_TIMEOUT);
	if (res != HAL_OK)
		return Bmp_Data_Transmit_Error;

	uint32_t tickstart = HAL_GetTick();

	do {
		// read result
		res = HAL_I2C_Master_Transmit(i2c, BMP_ADDRESS, to_transmit, 1,
		I2C_TIMEOUT);
		if (res != HAL_OK)
			return Bmp_Data_Transmit_Error;
		res = HAL_I2C_Master_Receive(i2c, BMP_ADDRESS, receive_data, 5,
		I2C_TIMEOUT);
		if (res != HAL_OK)
			return Bmp_Data_Transmit_Error;

		if ((HAL_GetTick() - tickstart) > BMP_TEMPERATURE_DELAY) {
			return Bmp_Timeout;
		}
	} while ((*msc >> 5) & 0x01);
	uint32_t ut = (*msb << 8) + *lsb;

	/*
	 * read pressure
	 */
	to_transmit[1] = BMP_MODE_PRESSURE;

	// start the measurement
	res = HAL_I2C_Master_Transmit(i2c, BMP_ADDRESS, to_transmit, 2,
	I2C_TIMEOUT);
	if (res != HAL_OK)
		return Bmp_Data_Transmit_Error;

	tickstart = HAL_GetTick();

	do {
		// read result
		res = HAL_I2C_Master_Transmit(i2c, BMP_ADDRESS, to_transmit, 1,
		I2C_TIMEOUT);
		if (res != HAL_OK)
			return Bmp_Data_Transmit_Error;
		res = HAL_I2C_Master_Receive(i2c, BMP_ADDRESS, receive_data, 5,
		I2C_TIMEOUT);
		if (res != HAL_OK)
			return Bmp_Data_Transmit_Error;

		if ((HAL_GetTick() - tickstart) > BMP_PRESSURE_DELAY) {
			return Bmp_Timeout;
		}
	} while ((*msc >> 5) & 0x01);

	int32_t up = (((*msb) << 16) + ((*lsb) << 8) + *xlsb) >> (8 - BMP_OSS);

	/*
	 * calculate
	 */
	// true temperature
	int32_t x1 = ((ut - bmp->_AC6) * bmp->_AC5) >> 15;
	int32_t x2 = (bmp->_MC << 11) / (x1 + bmp->_MD);
	int32_t b5 = x1 + x2;
	bmp->Temperature = (b5 + 8) >> 4;

	//true pressure
	int32_t b6 = b5 - 4000;
	x1 = (bmp->_B2 * (b6 * b6 >> 12)) >> 11;
	x2 = (bmp->_AC2 * b6) >> 11;
	int32_t x3 = x1 + x2;
	int32_t b3 = ((((bmp->_AC1 << 2) + x3) << BMP_OSS) + 2) >> 2;
	x1 = bmp->_AC3 * b6 >> 13;
	x2 = (bmp->_B1 * (b6 * b6 >> 12)) >> 16;
	x3 = (x1 + x2 + 2) >> 2;
	uint32_t b4 = bmp->_AC4 * (uint32_t) (x3 + 32768) >> 15;
	uint32_t b7 = ((uint32_t) up - b3) * (50000 >> BMP_OSS);
	int32_t p = (b7 < 0x80000000) ? (b7 << 1) / b4 : b7 / b4 << 2;
	x1 = p >> 8;
	x1 *= x1;
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	bmp->Pressure = p + ((x1 + x2 + 3791) >> 4);
	return Bmp_Ok;
}

BmpStatus BMP_180_Update(Bmp180 *bmp) {
	BmpStatus res = _update(bmp, bmp->_i2c);
	if (res != Bmp_Ok)
		return res;

	bmp->_p_last_3 = bmp->_p_last_2;
	bmp->_p_last_2 = bmp->_p_last_1;
	bmp->_p_last_1 = bmp->Pressure;
	bmp->Pressure = (bmp->_p_last_3 + bmp->_p_last_2 + bmp->_p_last_1
			+ bmp->_p_last_1) / 4;

	bmp->_t_last_3 = bmp->_t_last_2;
	bmp->_t_last_2 = bmp->_t_last_1;
	bmp->_t_last_1 = bmp->Temperature;
	bmp->Temperature = (bmp->_t_last_3 + bmp->_t_last_2 + bmp->_t_last_1
			+ bmp->_t_last_1) / 4;
	return res;
}
