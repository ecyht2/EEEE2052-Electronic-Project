#ifndef H62PEP_H
#define H62PEP_H

#include "arm_math.h"

/* This file and libh62pep.a provide code for you to use in your project
 * that initialises the ADC and runs a 512 point FFT on the data. You
 * should use it for the semester two work if you do not yet have your
 * own ADC code working.
 *
 * Call h62pep_init() after doing the rest of your initialisation, in
 * particular ensure you have configured the clock to high speed mode.
 * You should choose the GPIO port, pin and adc channel as appropriate.
 *
 * The resources used are:
 *
 * ADC: Channel and pin as chosen with h62pep_init().
 * Timers: Timer 4.
 *
 * h62pep_init() will return 1 if there was an error. On success it
 * returns 0.
 *
 * h62pep_speed() can be used to obtain a speed value. The speed in m/s
 * and the signal level (used to indicate amplitude) are returned in
 * the pointers speed_ms and signal_level.
 *
 * h62pep_speed() will return a 0 on success. If it returns a 1 that
 * indicates that there was no sample available and so you should not
 * use the values from speed_ms or signal_level.
 *
 * USING THIS LIBRARY
 * ==================
 *
 * To use this library, copy h62pep.h into your project include folder.
 * Copy libh62pep.a into your project folder.
 *
 * In SystemWorkbench, select your project properties. Expand the C/C++
 * Build section and choose Settings. In the Settings window choose
 * Libraries from the MCU GCC Linker section. Click the Add icon in the
 * Libraries section and add the entry h62pep. Now add an entry to the
 * Library Search Path section. If you have put libh62pep.a directly in
 * your project folder this can simply be ${ProjDirPath}
 *
 * In your own C source, #include the h62pep.h header and then use the
 * functions as you need.
 *
 * You will also need to configure the DSP library as described in the
 * "Using the DSP Library" document on moodle.
 */

// Example for the PA0 input: h62pep_init(GPIOA, GPIO_PIN_0, ADC_CHANNEL_5);

int h62pep_init(GPIO_TypeDef  *GPIOx, uint32_t adc_pin, uint32_t adc_channel);
int h62pep_speed(float32_t *speed_ms, float32_t *signal_level);

#endif
