/*
 * utilities.c
 *
 *  Created on: Dec 6, 2022
 *      Author: University
 */
#include "utilities.h"

#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>

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

/** @breif Converts a 2 digit decimal number into a BCD encoding.
 *
 * The BCD number is encoded in an 8-bit unsigned integer in which
 * the first 4 bits of the number represents the 10s and the last 4
 * bits of number represents the 1s.
 *
 * If the number given is greater than 99 0 will be returned.
 *
 * It uses the Double dabble algorithm to convert a binary number
 * to BCD.
 * https://en.wikipedia.org/wiki/Double_dabble
 *
 * @param number The number to convert to BCD.
 * @return The number encoded in BCD.
 * */
uint8_t binToBCD(uint8_t number) {
    uint8_t result = 0;

    // Returning 0 if number is 100 or above
    if (number > 99) {
      return 0;
    }

    while(number > 0) {
      // Getting the MSB
      if(number & 0x80) {
        result++;
      }
      // Shifting number
      number = number << 1;

      // Breaking out once it has shifted 8 times
      if(number == 0) {
        break;
      }

      // Adding 3 if any of the numbers are 5 or greater
      if((result & 0x0f) > 0x05) {
        result += 0x03;
      }

      if((result & 0xf0) > 0x50) {
        result += 0x30;
      }

      // Shifting output
      result = result << 1;
    }

    return result;
}
