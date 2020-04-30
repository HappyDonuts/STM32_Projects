/*
 * electronic_load.c
 *
 *  Created on: Mar 19, 2020
 *      Author: Javi
 */

/* INCLUDES -------------------------------- */
#include "electronic_load.h"

#include <math.h>

#include "mcp4725.h"
#include "ads115.h"
#include "ssd1306_basic.h"
/* PERIPHERALS ----------------------------- */
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

I2C_HandleTypeDef* i2c = &hi2c1;	//
TIM_HandleTypeDef* tim_oled = &htim2;
TIM_HandleTypeDef* tim_sense = &htim3;
/* DEFINES  -------------------------------- */

/* TYPEDEF --------------------------------- */

/* VARIABLES ------------------------------- */
mcp_t* mcp_1; //DAC module
ssd1306_t* ssd1306_1;
ads_t* ads_1;
/* FUNCTION PROTOTYPES --------------------- */

/**
  * @brief  The application entry point.
  * @retval none
  */
void main_s(void){
	/* MAIN CODE */
	mcp_1 = mcp_new(i2c, 0xC0); //0xC4 - other address
	ads_1 = ads_new(i2c, 0x48);
	ssd1306_1 = ssd1306_new(i2c, 0x79);

	mcp_write(mcp_1, 1000, 1);
	HAL_TIM_Base_Start_IT(tim_oled);
	
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
	if (htim == tim_oled){
		SSD1306_Putint(ssd1306_1, 15, NO, 1);
		SSD1306_UpdateScreen(ssd1306_1);
	}
	if (htim == tim_sense){

	}
}

