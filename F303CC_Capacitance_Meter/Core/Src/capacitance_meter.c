/*
 * capacitance_meter.c
 *
 *  Created on: May 15, 2020
 *      Author: Javi
 */

/* INCLUDES -------------------------------- */
#include "capacitance_meter.h"
/* PERIPHERALS ----------------------------- */
extern TIM_HandleTypeDef htim2;
TIM_HandleTypeDef* tim_c = &htim2;

GPIO_TypeDef* PORT_Button = GPIOA;
uint16_t PIN_Button = GPIO_PIN_8;

GPIO_TypeDef* PORT_LED = GPIOB;
uint16_t PIN_LED = GPIO_PIN_15;
/* DEFINES  -------------------------------- */

/* TYPEDEF --------------------------------- */

/* VARIABLES ------------------------------- */

/* FUNCTION PROTOTYPES --------------------- */


/**
  * @brief  The application entry point.
  * @retval none
  */
void main_s(void){
	/* MAIN CODE */
	
	
	while(1){
		/* WHILE CODE */
		
	}
}

/**
  * @brief  Period elapsed callback.
  * @retval none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/* TIM CALLBACK CODE */
	
}

/**
  * @brief  EXTI line detection callbacks.
  * @retval none
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* GPIO CALLBACK CODE */
	if (GPIO_Pin == PIN_Button){
		HAL_GPIO_TogglePin(PORT_LED, PIN_LED);
	}
}
