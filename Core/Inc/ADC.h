/*
 * ADC.h
 *
 *  Created on: 30 Nov 2022
 *      Author: wong80
 * 	   License: GPL-3.0-or-later
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#include <stdio.h>
#include <FFT.h>
#include <stdint.h>
#include "LCD_Display.h"

typedef struct {
	ADC_HandleTypeDef *hadc;

	/* PRIVATE */
	struct ADC_param ADC_val;
	struct FFT_res FFT_val;
	int flag;
} ADC;

void ADCInit(ADC *this, ADC_HandleTypeDef *hadc, const uint64_t adc_buf_len,
		uint32_t adc_buf[], uint32_t clock_speed, uint32_t prescaler,
		uint32_t bit, float sampling_time);
void ADCStart(ADC *this);
void ADCStop(ADC *this);
void ADCHandleCallback(ADC *this);
float ADCCalculateFrequency(ADC *this, int restart);

#endif /* SRC_ADC_H_ */
