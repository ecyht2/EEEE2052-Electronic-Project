/*
 * utilities.h
 *
 *  Created on: Dec 6, 2022
 *      Author: ecyht2
 * 	   License: GPL-3.0-or-later
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

#include "stm32l4xx.h"

static const unsigned int c = 1080000000;
static const unsigned int c_MPH = 671000000;

float calculateSpeed(float detected, float transmitted);
float calculateSpeedMPH(float detected, float transmitted);
void serial_print(UART_HandleTypeDef *huart, unsigned int buf_len, char *restrict format, ...);
uint8_t binToBCD(uint8_t number);

#endif /* INC_UTILITIES_H_ */
