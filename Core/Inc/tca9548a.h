/*
 * tca9548a.h
 *
 *  Created on: Mar 6, 2020
 *      Author: alexander
 */

#ifndef INC_TCA9548A_H_
#define INC_TCA9548A_H_

#include "stm32f1xx_hal.h"

typedef enum{
   TCA_B0 = 0x0,
   TCA_B1,
   TCA_B2,
   TCA_B3,
   TCA_B4,
   TCA_B5,
   TCA_B6,
   TCA_B7
} TcaAddress;

HAL_StatusTypeDef TCA_Set(I2C_HandleTypeDef *i2c, TcaAddress address);

#endif /* INC_TCA9548A_H_ */
