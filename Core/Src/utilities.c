/*
 * utilities.c
 *
 *  Created on: Dec 6, 2022
 *      Author: University
 */
#include "utilities.h"

float calculateSpeed(float detected, float transmitted) {
	float v = (float) c * detected / (2 * transmitted);
	return v;
}

float calculateSpeedMPH(float detected, float transmitted) {
	float v = (float) c_MPH * detected / (2 * transmitted);
	return v;
}
