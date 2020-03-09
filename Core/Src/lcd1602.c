/*
 * lcd1602.c
 *
 *  Created on: Mar 6, 2020
 *      Author: alexander
 */

#include "lcd1602.h"

#define LCD_I2C_ADDRESS 0x4E
#define LCD_TIMEOUT     1000
#define PIN_RS          0x01
#define PIN_EN          0x04
#define BACKLIGHT       0x08

#define LCD_DELAY_MS    2
#define I2C_TIMEOUT     1000

HAL_StatusTypeDef _lcd_send_internal_i2c(I2C_HandleTypeDef *i2c, uint8_t data,
		uint8_t flags) {

	while (HAL_I2C_IsDeviceReady(i2c, LCD_I2C_ADDRESS, 1, I2C_TIMEOUT) != HAL_OK)
		;

	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN;
	data_arr[1] = up | flags | BACKLIGHT;
	data_arr[2] = lo | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;

	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(i2c, LCD_I2C_ADDRESS,
			data_arr, 4,
			I2C_TIMEOUT);
	return res;
}

void _lcd_send_comand(I2C_HandleTypeDef *i2c, uint8_t cmd) {
	_lcd_send_internal_i2c(i2c, cmd, 0);
}

void _lcd_send_data(I2C_HandleTypeDef *i2c, uint8_t data) {
	_lcd_send_internal_i2c(i2c, data, PIN_RS);
}

void LCD_INIt_I2C(I2C_HandleTypeDef *i2c) {
	_lcd_send_comand(i2c, 0b00110000);
	_lcd_send_comand(i2c, 0b00000010);
	_lcd_send_comand(i2c, 0b00001100);
	_lcd_send_comand(i2c, 0b00000001);
}

void LCD_SendString(I2C_HandleTypeDef *i2c, char *str) {
	_lcd_send_comand(i2c, 0b10000000);
	while (*str) {
		if (*str == '\n')
			_lcd_send_comand(i2c, 0b11000000);
		else
			_lcd_send_data(i2c, (uint8_t) (*str));
		str++;
	}
}
