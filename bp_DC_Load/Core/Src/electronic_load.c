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
extern TIM_HandleTypeDef htim4;

I2C_HandleTypeDef* i2c = &hi2c1;	//
TIM_HandleTypeDef* tim_oled = &htim2;
TIM_HandleTypeDef* tim_sense = &htim3;
TIM_HandleTypeDef* tim_encoder = &htim4;
/* DEFINES  -------------------------------- */

/* TYPEDEF --------------------------------- */

/* VARIABLES ------------------------------- */
mcp_t* mcp_1; //DAC module
ssd1306_t* ssd1306_1; // OLED display
ads_t* ads_1; // ADC module
ads_t* ads_2; // ADC module

double i_set;
double i_load;
double v_in;
double v_amplified[40];
double v_load_array[40];
double v_load;

int16_t n_pulse = 0;

const double divider = 10.0909;
const double rsens = 0.05458;
const double factor = 1.11;
const double gain_sens = 11.3684;
/* FUNCTION PROTOTYPES --------------------- */

/**
  * @brief  The application entry point.
  * @retval none
  */
void main_s(void){
	/* MAIN CODE */
	mcp_1 = mcp_new(i2c, 0xC0); //0xC4 - other address
	ads_1 = ads_new(i2c, 0x48);
	ads_2 = ads_new(i2c, 0x49);
	ssd1306_1 = ssd1306_new(i2c, 0x79);


	HAL_TIM_Base_Start_IT(tim_sense);
	HAL_TIM_Base_Start_IT(tim_oled);
	HAL_TIM_Encoder_Start(tim_encoder, TIM_CHANNEL_2);

	while(1){
		/* WHILE CODE */
		n_pulse = (TIM4->CNT);
		n_pulse = n_pulse/4;
		if(n_pulse < 0){
			n_pulse = 0;
			TIM4->CNT = 0;
		}
		if (n_pulse > 50){
			n_pulse = 50;
			TIM4->CNT = 200;
		}

		HAL_Delay(50);
		i_set = n_pulse*100;
		v_in = i_set*divider*rsens/factor;
		mcp_write(mcp_1, v_in*4095/3300, 1);
	}
}

/**
  * @brief  Period elapsed callback.
  * @retval none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/* TIM CALLBACK CODE */
	if (htim == tim_oled){
		SSD1306_Putint(ssd1306_1, i_set, MA, 1);
		SSD1306_Putint(ssd1306_1, i_load, MA, 2);
		SSD1306_Putdouble(ssd1306_1, i_load*v_load/1000, 2, " W", 3);
		SSD1306_Putdouble(ssd1306_1, v_load, 2, V, 4);
//		SSD1306_Putint(ssd1306_1, v_sens, MV, 5);
		SSD1306_UpdateScreen(ssd1306_1);
	}
	if (htim == tim_sense){
		static uint8_t index_adc = 0 ;
		v_amplified[index_adc] = ads_read(ads_1, 5, 1);
		v_load_array[index_adc] = ads_read(ads_2, 4, 0);
		index_adc++;

		if (index_adc == 40){
			double mean_i = 0;
			double mean_v = 0;
			for(uint8_t i=0;i<40;i++){
				mean_i += v_amplified[i];
				mean_v += v_load_array[i];
			}
			double v_amp_mean = mean_i/40*1.13;
			double v_sens = v_amp_mean/gain_sens;
			i_load = v_sens/rsens;
			if (i_load < 100){
				i_load = 0;
			}
			v_load = (mean_v/40)*6/1000;
			index_adc = 0;
		}
	}
}

