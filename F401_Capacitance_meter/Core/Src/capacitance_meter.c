/*
 * capacitance_meter.c
 *
 *  Created on: Jun 8, 2020
 *      Author: Javi
 */

/* INCLUDES -------------------------------- */
#include "capacitance_meter.h"
#include "ssd1306_basic.h"
/* PERIPHERALS ----------------------------- */
extern I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef* i2c = &hi2c2;

extern TIM_HandleTypeDef htim2;
TIM_HandleTypeDef* tim_charge = &htim2;

GPIO_TypeDef* PORT_Q1 = GPIOB;
uint16_t PIN_Q1 = GPIO_PIN_9;

GPIO_TypeDef* PORT_Q2 = GPIOB;
uint16_t PIN_Q2 = GPIO_PIN_8;

GPIO_TypeDef* PORT_Q3 = GPIOB;
uint16_t PIN_Q3 = GPIO_PIN_7;

GPIO_TypeDef* PORT_Q4 = GPIOB;
uint16_t PIN_Q4 = GPIO_PIN_6;

GPIO_TypeDef* PORT_Q5 = GPIOB;
uint16_t PIN_Q5 = GPIO_PIN_5;

GPIO_TypeDef* PORT_Q6 = GPIOB;
uint16_t PIN_Q6 = GPIO_PIN_4;
/* DEFINES  -------------------------------- */

/* TYPEDEF --------------------------------- */

/* VARIABLES ------------------------------- */
ssd1306_t* display;
uint16_t cycles = 0;
/* FUNCTION PROTOTYPES --------------------- */


/**
  * @brief  The application entry point.
  * @retval none
  */
void main_s(void){
	/* MAIN CODE */
	display = ssd1306_new(i2c, 0x78); // 0x79
	HAL_TIM_Base_Start_IT(tim_charge);
	HAL_Delay(500);
	HAL_TIM_Base_Stop_IT(tim_charge);
	double counter = tim_charge->Instance->CNT;

	SSD1306_Putint(display, counter,  NO, 1);
	SSD1306_Putint(display, cycles,  NO, 2);
	SSD1306_UpdateScreen(display);

//	HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 1);
//	HAL_GPIO_WritePin(PORT_Q3, PIN_Q3, 1);
//	HAL_Delay(2000);
//
//	HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 0);
//	HAL_GPIO_WritePin(PORT_Q3, PIN_Q3, 0);
//	HAL_Delay(2000);

	while(1){
		/* WHILE CODE */
//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 0);
//		HAL_GPIO_WritePin(PORT_Q2, PIN_Q2, 0);
//		HAL_Delay(2000);

//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 1);
//		HAL_GPIO_WritePin(PORT_Q2, PIN_Q2, 1);
//		HAL_Delay(2000);
//
//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 0);
//		HAL_GPIO_WritePin(PORT_Q3, PIN_Q3, 0);
//		HAL_Delay(2000);

//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 1);
//		HAL_GPIO_WritePin(PORT_Q3, PIN_Q3, 1);
//		HAL_Delay(2000);

//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 0);
//		HAL_GPIO_WritePin(PORT_Q4, PIN_Q4, 0);
//		HAL_Delay(2000);
//
//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 1);
//		HAL_GPIO_WritePin(PORT_Q4, PIN_Q4, 1);
//		HAL_Delay(2000);
//
//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 0);
//		HAL_GPIO_WritePin(PORT_Q5, PIN_Q5, 0);
//		HAL_Delay(2000);
//
//		HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 1);
//		HAL_GPIO_WritePin(PORT_Q5, PIN_Q5, 1);
//		HAL_Delay(2000);
	}
}
//
/**
  * @brief  Period elapsed callback.
  * @retval none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/* TIM CALLBACK CODE */
	if (htim == tim_charge){
		static uint8_t ready = 0;
		if (ready == 1){
			cycles++;
		}
		ready = 1;
	}
}
//
///**
//  * @brief  EXTI line detection callbacks.
//  * @retval none
//  */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	/* GPIO CALLBACK CODE */
//
//}
