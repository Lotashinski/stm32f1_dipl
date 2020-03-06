/*
 * lcd1602.h
 *
 *  Created on: Mar 6, 2020
 *      Author: alexander
 */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

#include "stm32f1xx_hal.h"

void LCD_SendString(I2C_HandleTypeDef *i2c, char *str);
void LCD_INIt_I2C(I2C_HandleTypeDef *i2c);

#endif /* INC_LCD1602_H_ */
