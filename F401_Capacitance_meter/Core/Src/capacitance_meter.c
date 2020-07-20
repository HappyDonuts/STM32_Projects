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

GPIO_TypeDef* PORT_COMP_1 = GPIOB;
uint16_t PIN_COMP_1 = GPIO_PIN_14;

GPIO_TypeDef* PORT_COMP_2 = GPIOB;
uint16_t PIN_COMP_2 = GPIO_PIN_13;
/* DEFINES  -------------------------------- */
#define F_CLK 84000000
/* TYPEDEF --------------------------------- */

/* VARIABLES ------------------------------- */
ssd1306_t* display;
uint8_t status;
uint16_t cycles = 0;
double counter = 0;
uint32_t ch_sel;

channel_t* channels[5];
/* FUNCTION PROTOTYPES --------------------- */
uint8_t select_ch(void);
channel_t* channel_new(uint8_t num, GPIO_TypeDef* port, uint16_t pin);
uint32_t charge_cap(channel_t* ch);
void discharge_cap(uint16_t delay);
void display_cap(uint32_t* values, uint8_t size);

/**
  * @brief  The application entry point.
  * @retval none
  */
void main_s(void){
	/* MAIN CODE */
	display = ssd1306_new(i2c, 0x78); // 0x79

	channels[0] = channel_new(1, PORT_Q1, PIN_Q1);
	channels[1] = channel_new(2, PORT_Q2, PIN_Q2);
	channels[2] = channel_new(3, PORT_Q3, PIN_Q3);
	channels[3] = channel_new(4, PORT_Q4, PIN_Q4);
	channels[4] = channel_new(5, PORT_Q5, PIN_Q5);

	HAL_TIM_Base_Start_IT(tim_charge);
	HAL_TIM_Base_Stop_IT(tim_charge);

	uint8_t size = 1;
	uint32_t values[size];

	discharge_cap(500);
	ch_sel = select_ch();
	for (uint8_t i=0; i<size; i++){
		values[i] = charge_cap(channels[ch_sel]);
	}
	display_cap(values, size);

	while(1){
		/* WHILE CODE */

	}
}
uint8_t select_ch(void){
	charge_cap(channels[4]);
	if (counter > 168000){
		return 4;
	}
	charge_cap(channels[3]);
	if (counter > 16800){
		return 3;
	}
	charge_cap(channels[2]);
	if (counter > 16800){
		return 2;
	}
//	pulses = charge_cap(channels[1]);
//	if (pulses > 16800){
//		return 1;
//	}
	else {
		return 1;
	}
}

channel_t* channel_new(uint8_t num, GPIO_TypeDef* port, uint16_t pin){
	channel_t* ch = malloc(sizeof(*ch));
	ch->port = port;
	ch->pin = pin;
	return ch;
}

uint32_t charge_cap(channel_t* ch){
	HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 0);
	HAL_Delay(1);
	tim_charge->Instance->CNT = 0;
	cycles = 0;
	HAL_GPIO_WritePin(ch->port, ch->pin, 0);
	HAL_TIM_Base_Start_IT(tim_charge);

	while(status != CHARGED){
		HAL_Delay(1);
	}
	counter = (tim_charge->Instance->CNT + cycles*65000);
	double time_constant = counter/F_CLK;
	double delay_ms = 1+6*1000*time_constant;
	discharge_cap(delay_ms);
	return counter;
}

void discharge_cap(uint16_t delay){
	HAL_GPIO_WritePin(PORT_Q1, PIN_Q1, 1);
	HAL_GPIO_WritePin(PORT_Q2, PIN_Q2, 1);
	HAL_GPIO_WritePin(PORT_Q3, PIN_Q3, 1);
	HAL_GPIO_WritePin(PORT_Q4, PIN_Q4, 1);
	HAL_GPIO_WritePin(PORT_Q5, PIN_Q5, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(PORT_Q6, PIN_Q6, 1);
	HAL_Delay(delay);
	status = DISCHARGED;
}

void display_cap(uint32_t* values, uint8_t size){
	double mean;
	for (uint8_t i=0; i<size; i++){
		mean = values[i];
	}
	mean = mean/size;

	switch (ch_sel){
		case 0:
			mean = mean/84;
			if (mean > 1000){
				mean = mean/1000;
				SSD1306_Putdouble(display, mean, 1, "nF", 1);
			} else {
				SSD1306_Putdouble(display, mean, 1, "pF", 1);
			}
			break;
		case 1:
			mean = counter/(84*100);
			SSD1306_Putdouble(display, mean, 1, "nF", 1);
			break;
		case 2:
			mean = counter/(84*10);
			SSD1306_Putdouble(display, mean, 1, "nF", 1);
			break;
		case 3:
			mean = counter/84;
			if (mean > 1000){
				mean = mean/1000;
				SSD1306_Putdouble(display, mean, 1, "uF", 1);
			} else {
				SSD1306_Putdouble(display, mean, 1, "nF", 1);
			}
			break;
		case 4:
			mean = counter/(84*100);
			SSD1306_Putdouble(display, mean, 1, "uF", 1);
	}
	SSD1306_UpdateScreen(display);
}
/**
  * @brief  Period elapsed callback.
  * @retval none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/* TIM CALLBACK CODE */
	if (htim == tim_charge){
		cycles++;
	}
}

/**
  * @brief  EXTI line detection callbacks.
  * @retval none
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* GPIO CALLBACK CODE */
	if (GPIO_Pin == GPIO_PIN_14){
		HAL_TIM_Base_Stop_IT(tim_charge);
		status = CHARGED;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	}
}
