/*
 * capacitance_meter.h
 *
 *  Created on: Jun 8, 2020
 *      Author: Javi
 */

#ifndef INC_CAPACITANCE_METER_H_
#define INC_CAPACITANCE_METER_H_

#include "main.h"

typedef enum cap_status{
	DISCHARGED,
	CHARGED
} cap_status;

typedef struct channel_t{
	GPIO_TypeDef* port;
	uint16_t pin;
} channel_t;

void main_s(void);

#endif /* INC_CAPACITANCE_METER_H_ */
