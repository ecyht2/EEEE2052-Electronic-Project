/*
 * utilities.c
 *
 *  Created on: Dec 6, 2022
 *      Author: University
 */
#include "utilities.h"

#include <stdio.h>

#include "stm32l4xx.h"

float calculateSpeed(float detected, float transmitted) {
	float v = (float) c * detected / (2 * transmitted);
	return v;
}

float calculateSpeedMPH(float detected, float transmitted) {
	float v = (float) c_MPH * detected / (2 * transmitted);
	return v;
}

/**
 * @breif Prints the string in the serial output.
 *
 * *Note*: The maximum number of characters that can be print is 50
 * including the end of end of string character.
 *
 * @param str The string to print.
 * @param huart The serial port to use.
 */
void serial_print(char *str, UART_HandleTypeDef *huart) {
	char uart_buf[50];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, str);
	HAL_UART_Transmit(huart, (unsigned char*) uart_buf, uart_buf_len, 100);
}
