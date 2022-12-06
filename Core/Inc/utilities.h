/*
 * utilities.h
 *
 *  Created on: Dec 6, 2022
 *      Author: University
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

#include "stm32l4xx.h"

const unsigned int c = 1080000000;
const unsigned int c_MPH = 671000000;

float calculateSpeed(float detected, float transmitted);
float calculateSpeedMPH(float detected, float transmitted);
void serial_print(char*, UART_HandleTypeDef*);

#endif /* INC_UTILITIES_H_ */
