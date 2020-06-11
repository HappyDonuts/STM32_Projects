/*
 * function_generator.c
 *
 *  Created on: May 14, 2020
 *      Author: Javi
 */

/* INCLUDES -------------------------------- */
#include "function_generator.h"
#include "mcp4922.h"
#include "math.h"
/* PERIPHERALS ----------------------------- */
extern SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef* spi = &hspi1;

GPIO_TypeDef* PORT_CS = GPIOB;
uint16_t PIN_CS = GPIO_PIN_0;

extern TIM_HandleTypeDef htim2;
TIM_HandleTypeDef* tim_sample = &htim2;
/* DEFINES  -------------------------------- */
#define SIZE 10
/* TYPEDEF --------------------------------- */

/* VARIABLES ------------------------------- */
mcp4922_t* dac_1;
uint8_t index_sample = 0;
uint16_t signal[SIZE];
/* FUNCTION PROTOTYPES --------------------- */
void setSignal(void);

/**
  * @brief  The application entry point.
  * @retval none
  */
void main_s(void){
	/* MAIN CODE */
	dac_1 = mcp4922_new(spi, PORT_CS, PIN_CS, 3300, 3300, 1);
	setSignal();
	HAL_TIM_Base_Start_IT(tim_sample);
	while(1){
		/* WHILE CODE */
//		mcp4922_write(dac_1, 0, 0);
//		HAL_Delay(2);
//		mcp4922_write(dac_1, 3300, 0);
//		HAL_Delay(2);
	}
}

void setSignal(void){
	for (uint16_t i=0;i<SIZE;i++){
		float voltage = (sin(2*M_PI*i/SIZE)*3300/2)+3300/2;
		signal[i] = voltage;
	}
}

/**
  * @brief  Period elapsed callback.
  * @retval none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/* TIM CALLBACK CODE */
	if (htim == tim_sample){
		mcp4922_write(dac_1, signal[index_sample], 0);
		index_sample++;
		if (index_sample == SIZE){
			index_sample = 0;
		}
	}
}

/**
  * @brief  EXTI line detection callbacks.
  * @retval none
  */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	/* GPIO CALLBACK CODE */
//
//}
