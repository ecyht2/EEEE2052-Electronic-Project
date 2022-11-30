/*
 * ADC.c
 *
 *  Created on: Nov 30, 2022
 *      Author: zhiyu
 */
#include "ADC.h"
#include "main.h"
#include <stdio.h>
#include <FFT.h>
#include <stdint.h>
#include <stdio.h>
#include "LCD_Display.h"


int ADC (int *flag, struct ADC_param *ADC_val, struct FFT_res *FFT_val)

{
	 char uart_buf[50];
	 int uart_buf_len;


	start_FFT(flag, ADC_val, FFT_val);
	uart_buf_len = sprintf(uart_buf, "ADC f: %lf", FFT_val->fdominant);

	return uart_buf_len;
}

