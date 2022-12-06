/*
 * utilities.h
 *
 *  Created on: Dec 6, 2022
 *      Author: University
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

const unsigned int c = 1080000000;
const unsigned int c_MPH = 671000000;

float calculateSpeed(float detected, float transmitted);
float calculateSpeedMPH(float detected, float transmitted);

#endif /* INC_UTILITIES_H_ */
