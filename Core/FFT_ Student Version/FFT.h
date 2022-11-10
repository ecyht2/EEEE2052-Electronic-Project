/*
 *Copyright (c) 2022, Ms. Ho Yen Rou
 *@file: FFT.h
 *@brief: STM32L476RG FFT library. Please refer to README file (FFT Library.md) for further steps and explanation
 *@brief: This is designed to detect frequency from 50Hz to 1500Hz. However, frequency larger than 1500Hz can also be detected depending on the sampling frequency.
 *@brief: Sampling frequency is checked to ensure each frequency bin <50Hz
 *@brief: FFT samples are collected using DMA method.
 * 
 * If you have any enquiry on the library file, please contact PuChuan.Hsian@nottingham.edu.my
 */

#ifndef SRC_FFT_H_
#define SRC_FFT_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h" 			//used as it contains FFT functions

/* USER CODE END Includes */

/* Private USER DEFINE ----------------------------------------------------------*/
/* USER CODE BEGIN DEFINE */
typedef struct ADC_param
{
	uint32_t *adc_buf;			//pointer to buffer
	uint32_t adc_buf_len;		//buffer length, either 32,64,128,256,512,1024,2048,or 4096
	uint32_t speed;				//clock speed
	uint32_t prescaler;			//ADC prescaler
	uint32_t bit;				//ADC bit resolution
	float sampling_time;		//ADC sampling time
} ADC_var;

typedef struct FFT_res
{
	float offset;				//sampling offset
	float fsampling;			//sampling frequency
	float fbin;					//frequency bin
	float fdominant;			//dominant frequency
	float magnitude;			//dominant frequency magnitude
	uint32_t index;				//FFT dominant frequency index
} FFT_var;

/* USER CODE END DEFINE*/


/* Private functions----------------------------------------------*/
/**
*@brief: start_FFT()  	Start Fast Fourier Transform algorithm.
*@param: flag  			Check if complete callback interrupt is called
*@param: ADC_var		Store clock values
*@param: FFT_var		Store FFT variables
*@retval: 0 if there's error, 1 if there's no error
*/
int start_FFT(int *flag, struct ADC_param *ADC_var, struct FFT_res *FFT_var);

/*---------------------------------------------------------------*/
/**
 * Doppler radar sensor (DRS)
*@brief: get_velocity() calculates object velocity based on DRS signal.
*@param: fdominant 		DRS return  frequency;
*@param: fwave 			DRS transmitted frequency;
*@param: velocity 		Stores the calculation result
*@retval: none
*/

void get_velocity(float fdominant, float fwave, float * velocity);


/*---------------------------------------------------------------*/
#endif /* SRC_FFT_H_ */
