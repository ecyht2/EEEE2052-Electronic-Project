/*
 * utilities.c
 *
 *  Created on: Dec 6, 2022
 *      Author: University
 */
#include "utilities.h"

#include <stdio.h>
#include <stdarg.h>

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
void serial_print(UART_HandleTypeDef *huart, unsigned int buf_len, char *restrict format, ...) {
	char uart_buf[buf_len];
	int uart_buf_len;

	va_list args;
	va_start(args, format);

	uart_buf_len = vsnprintf(uart_buf, buf_len, format, args);
	HAL_UART_Transmit(huart, (unsigned char*) uart_buf, uart_buf_len, 100);
}
