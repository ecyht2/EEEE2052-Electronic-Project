#Copyright (c) 2022, Ms. Ho Yen Rou
#If you have any enquiry on the library file, please contact #PuChuan.Hsian@nottingham.edu.my
# **FFT.h Guide**
***
## Background
This library is made for STM32L476RG Fast Fourier Transform (FFT) applications.

***
## Limitations
1. Only certain buffer length for the FFT algorithm is acceptable. They are 32, 64, 128, 256, 512, 1024, 2048, or 4096.
2. The frequency bin value depends on the setup in .ioc file. The frequency bin is calculated using the formula below:
    *bin=ADC clock speed / (ADC bit resolution+ ADC sampling time) / ADC prescaler / ADC buffer length* 

***
## Requirements before using fft.h
1. To enable float in stm32, please refer to https://github.com/ethanhuanginst/STM32CubeIDE-Workshop-2019/blob/master/hands-on/03_printf/printf_float_number.md
2. To include the DSP libraries, please refer to the link below. Log in with your STM32 account to view the article. https://community.st.com/s/article/configuring-dsp-libraries-on-stm32cubeide
3. To include the FFT.h library, please refer to this tutorial: https://www.youtube.com/watch?v=ab9Tj2LbRUo&t=304s
***
## Custom Structures
#### 1. ADC_param
Store ADC parameters.
```C
typedef struct ADC_param
{
	uint32_t *adc_buf;			//pointer to buffer
	uint32_t adc_buf_len;		//buffer length, either 32,64,128,256,512,1024,2048,or 4096
	uint32_t speed;				//clock speed
	uint32_t prescaler;			//ADC prescaler
	uint32_t bit;				//ADC bit resolution
	float sampling_time;		//ADC sampling time
} ADC_var;
```
#### 2. FFT_res
Store FFT variables.
```C
typedef struct FFT_res
{
	float offset;				//sampling offset
	float fsampling;			//sampling frequency
	float fbin;					//frequency bin
	float fdominant;			//dominant frequency
	float magnitude;			//dominant frequency magnitude
	uint32_t index;				//FFT dominant frequency index
} FFT_var;
```
***
## Functions

#### 1. start_FFT( )
Execute the whole Fast Fourier Transform algorithm.  Returns 0 if there's error, 1 if there's no error.
```C
int start_FFT(int *flag, struct ADC_param *ADC_var, struct FFT_res *FFT_var);
```
>*\*flag:* Check if complete callback interrupt is called
*\*ADC_var:* Store clock values
*\*FFT_var:* Store FFT variables
#### 2. get_velocity( )
Calculates object velocity based on Doppler Radar Sensor Signal.
```C
void get_velocity(float fdominant, float fwave, float * velocity)
```
>*fdominant:* Doppler Radar Sensor return frequency
*fwave:* Doppler Radar Sensor transmitted frequency
*\*velocity:* Stores the calculation result
***
## Example: Sample Doppler Radar Sensor Signal through DMA and start FFT
#### Setup
Initialise a global variable flag.
```C
int flag=0;
```
#### main function 
First, initialise the custom structures.
```C
struct ADC_param ADC_val={0};
struct FFT_res FFT_val={0};
```
Next, type in all relevant ADC parameters and initialise a new buffer array to store ADC inputs.
```C
ADC_val.bit=12;
ADC_val.prescaler=64;
ADC_val.sampling_time=47.5;
ADC_val.speed=80000000;					//clock speed on .ioc file
ADC_val.adc_buf_len=2048;				//buffer length
uint32_t adc_buf[ADC_val.adc_buf_len];	//buffer array
ADC_val.adc_buf=adc_buf;				//store the value of buffer array pointer
```


After that, start to sample the data through DMA. Then, store the data inside the buffer: adc_buf.
```C
HAL_ADC_Start_DMA(&hadc1, adc_buf,ADC_val.adc_buf_len);
```

Next, in the while loop, if flag ==1, start the FFT algorithm.
```C
start_FFT(&flag,&ADC_val,&FFT_val);
```
You can execute other commands as you wish, and when you want to start the sampling process for the FFT again, start the DMA again.
```C
HAL_ADC_Start_DMA(&hadc1, adc_buf,ADC_BUF_LEN);
```

Next, when the DMA buffer is completely filled, the complete callback function is executed. 
#### HAL_ADC_ConvCpltCallback() function
Declare this function after main function. This function is a built in interrupt that will be called when the buffer is completely filled. 

Inside the function, first, the flag is updated to 1.
```C
flag=1;
```
Then, the DMA sampling process is stopped to prevent buffer overflowing, and to stop the interrupt to keep on being called.
```C
HAL_ADC_Stop_DMA(&hadc1);
```

***

