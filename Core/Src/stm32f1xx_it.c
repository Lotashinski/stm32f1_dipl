/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

#include "stdlib.h"
#include "string.h"
#include "bmp180.h"
#include "tca9548a.h"
#include "lcd1602.h"
#include "ssd1306.h"

extern uint8_t requestCode[];
extern Bmp180 bmp[6];
extern I2C_HandleTypeDef hi2c1;
uint8_t package[8];

void transmit(uint64_t data) {
	for (uint8_t i = 0; i < 8; ++i) {
		package[i] = (uint8_t) data;
		data >>= 8;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) package, 8, 0xFFFF);
}

void transmitter(uint8_t i) {
	TCA_Set(&hi2c1, (TcaAddress) i);
	BMP_180_Update(&bmp[i]);
	uint32_t _t = (bmp[i].isInit << 31) | bmp[i].Temperature;
	uint64_t data = (((uint64_t) _t << 32)) | bmp[i].Pressure;
	transmit(data);
}

void case0() {
	transmitter(0);
}

void case1() {
	transmitter(1);
}

void case2() {
	transmitter(2);
}

void case3() {
	transmitter(3);
}

void case4() {
	transmitter(4);
}

void case5() {
	transmitter(5);
}

void case6() {
	transmitter(6);
}

void case80() {
	transmit(0);
}

void caseDefault() {
	transmit(-1);
}
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) {
	/* USER CODE BEGIN TIM2_IRQn 0 */

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	ssd1306_Fill(White);
	char info[6][17] = { { 'A', '-', 0 }, { 'B', '-', 0 }, { 'C', '-', 0 }, {
			'D', '-', 0 }, { 'E', '-', 0 }, { 'F', '-', 0 } };

	for (uint8_t i = 0; i < 6; ++i) {
		TCA_Set(&hi2c1, (TcaAddress) i);
		BMP_180_Update(&bmp[i]);
		char data_p[7] = { 0 };
		if (bmp[i].isInit == 1) {
			itoa(bmp[i].Pressure, data_p, 10);
			if (bmp[i].Pressure < 100000) {
				strcat(info[i], " ");
			}
			strcat(info[i], data_p);
			itoa(bmp[i].Temperature, data_p, 10);
			strcat(info[i], " ");
			if (bmp[i].Temperature < 100) {
				strcat(info[i], " ");
			}
			if (bmp[i].Temperature < 10) {
				strcat(info[i], " ");
			}
			strcat(info[i], data_p);
		} else {
			strcat(info[i], "invalid");
		}
		ssd1306_SetCursor(4, i * 10);
		ssd1306_WriteString(info[i], Font_7x9, Black);
	}

	ssd1306_UpdateScreen();
	/* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */
	HAL_GPIO_WritePin(GPIOC, 13, SET);
	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	switch (requestCode[0]) {
	case 0:
		case0();
		break;
	case 1:
		case1();
		break;
	case 2:
		case2();
		break;
	case 3:
		case3();
		break;
	case 4:
		case4();
		break;
	case 5:
		case5();
		break;
	case 6:
		case6();
		break;
	case 80:
		case80();
		break;
	default:
		caseDefault();
	}
	HAL_GPIO_WritePin(GPIOC, 13, RESET);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) requestCode, 1);
	/* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
