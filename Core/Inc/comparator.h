/*
 * comparator.h
 *
 *  Created on: Nov 30, 2022
 *      Author: University
 */

#ifndef SRC_COMPARATOR_H_
#define SRC_COMPARATOR_H_

#include <inttypes.h>

#include "main.h"

void handleClockCallback(COMP_HandleTypeDef *hcomp, uint8_t *comp_val, uint64_t *ticks, uint64_t *clock_ticks);
float calculateFrequency(uint64_t ticks, uint64_t clock_ticks, float tick_period);

#endif /* SRC_COMPARATOR_H_ */
