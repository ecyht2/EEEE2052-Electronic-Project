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


void ADCInit(ADC *this, const uint64_t adc_buf_len, uint32_t adc_buf[], uint32_t clock_speed,
	     uint32_t prescaler, uint32_t bit, float sampling_time) {
  this->ADC_val.adc_buf_len = adc_buf_len;
  this->ADC_val.adc_buf = adc_buf;
  this->ADC_val.speed = clock_speed;
  this->ADC_val.prescaler = prescaler;
  this->ADC_val.bit = bit;
  this->ADC_val.sampling_time = sampling_time;

  ADCStart(this);
}

void ADCStart(ADC *this) {
  HAL_ADC_Start_DMA(this->hadc, this->ADC_val.adc_buf, this->ADC_val.adc_buf_len);
}

void ADCStop(ADC *this) {
  HAL_ADC_Stop_DMA(this->hadc);
}

void ADCHandleCallback(ADC *this) {
  this->flag = 1;
  ADCStop(this);
}

float ADCCalculateFrequency(ADC *this, int restart) {
  // Doing FFT
  if (this->flag == 1) {
      start_FFT(&(this->flag), &(this->ADC_val), &(this->FFT_val));
  }
  // Restarting ADC DMA
  if (restart == 1) {
      ADCStart(this);
  }
  return this->FFT_val.fdominant;
}

