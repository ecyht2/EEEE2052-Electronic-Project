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
 * *Note*: Please ensure a sufficient amount of buf_len is
 * given for the string to print correct.
 *
 * @param huart The serial port to use.
 * @param buf_len The size of the buffer to print.
 * @param format The format string.
 */
void serial_print(UART_HandleTypeDef *huart, unsigned int buf_len, char *restrict format, ...) {
	char uart_buf[buf_len];
	int uart_buf_len;

	va_list args;
	va_start(args, format);

	uart_buf_len = vsnprintf(uart_buf, buf_len, format, args);
	HAL_UART_Transmit(huart, (unsigned char*) uart_buf, uart_buf_len, 100);
}
