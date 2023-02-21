/*
 * comparator.h
 *
 *  Created on: Nov 30, 2022
 *      Author: ecyht2
 * 	   License: GPL-3.0-or-later
 */

#ifndef SRC_COMPARATOR_H_
#define SRC_COMPARATOR_H_

#include <inttypes.h>

#include "main.h"

typedef struct {
	COMP_HandleTypeDef *hcomp;
	TIM_HandleTypeDef *htim;
	float timer_frequency;

	/* PRIVATE */
	uint8_t current_comp_val;
	uint64_t ticks;
	uint64_t clock_ticks;
} Comparator;

void comparatorInit(Comparator *this, COMP_HandleTypeDef *hcomp,
		TIM_HandleTypeDef *htim, float timer_frequency);
void comparatorStart(Comparator *this);
void comparatorStop(Comparator *this);
void comparatorHandleClockCallback(Comparator *this);
float comparatorCalculateFrequency(Comparator *this);

#endif /* SRC_COMPARATOR_H_ */
