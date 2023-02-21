/*
 * comparator.cpp
 *
 *  Created on: Nov 30, 2022
 *      Author: ecyht2
 * 	   License: GPL-3.0-or-later
 */
#include "comparator.h"

#include <inttypes.h>

#include "main.h"

void comparatorInit(Comparator *this, COMP_HandleTypeDef *hcomp,
		TIM_HandleTypeDef *htim, float timer_frequency) {
	this->hcomp = hcomp;
	this->htim = htim;
	this->timer_frequency = timer_frequency;
}

void comparatorStart(Comparator *this) {
	HAL_COMP_Start(this->hcomp);
	HAL_TIM_Base_Start_IT(this->htim);
	this->current_comp_val = HAL_COMP_GetOutputLevel(this->hcomp);
}

void comparatorStop(Comparator *this) {
	HAL_COMP_Stop(this->hcomp);
	HAL_TIM_Base_Stop_IT(this->htim);
}

void comparatorHandleClockCallback(Comparator *this) {
	COMP_HandleTypeDef *hcomp = this->hcomp;
	uint8_t *comp_val = &(this->current_comp_val);
	uint64_t *ticks = &(this->ticks);
	uint64_t *clock_ticks = &(this->clock_ticks);

	// Resetting values
	if (*clock_ticks > 65535) {
		*ticks = 0;
		*clock_ticks = 0;
	}

	// Increasing values
	// Reading comparator
	uint8_t current_comp_val = HAL_COMP_GetOutputLevel(hcomp);
	// Checking if the value changed
	if (current_comp_val != *comp_val) {
		(*ticks)++;
		*comp_val = current_comp_val;
	}
	(*clock_ticks)++;
}

float comparatorCalculateFrequency(Comparator *this) {
	float comp_freq = this->timer_frequency * (float) this->ticks
			/ (float) this->clock_ticks / 2;
	return comp_freq;
}
