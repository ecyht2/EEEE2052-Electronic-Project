/*
 * comparator.cpp
 *
 *  Created on: Nov 30, 2022
 *      Author: University
 */
#include "comparator.h"

#include <inttypes.h>

#include "main.h"

void handleClockCallback(COMP_HandleTypeDef *hcomp, uint8_t *comp_val, uint64_t *ticks, uint64_t *clock_ticks) {
  // Resetting values
  if (*clock_ticks > 65535) {
	  *ticks = 0;
	  *clock_ticks = 0;
  }

  // Increasing values
  uint8_t current_comp_val = HAL_COMP_GetOutputLevel(hcomp);
  if (current_comp_val != *comp_val) {
	  (*ticks)++;
	  *comp_val = current_comp_val;
  }
  (*clock_ticks)++;
}

float calculateFrequency(uint64_t ticks, uint64_t clock_ticks, float tick_period) {
  float comp_freq = (float) ticks / (float) (clock_ticks * tick_period) / 2;
  return comp_freq;
}
