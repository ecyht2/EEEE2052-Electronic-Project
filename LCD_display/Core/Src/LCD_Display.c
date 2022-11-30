/*
 * LCD_Display.c
 *
 *  Created on: Nov 7, 2022
 *      Author: User
 */
#include "main.h"
#include <stdio.h>
#include "myheader.h"

#define D4_PORT GPIOB
#define D4_PIN GPIO_PIN_5
#define D5_PORT GPIOB
#define D5_PIN GPIO_PIN_4
#define D6_PORT GPIOB
#define D6_PIN GPIO_PIN_10
#define D7_PORT GPIOA
#define D7_PIN GPIO_PIN_8
#define RS_PORT GPIOA
#define RS_PIN GPIO_PIN_9
#define EN_PORT GPIOC
#define EN_PIN GPIO_PIN_7
#define A0_PORT GPIOA
#define A0_PIN GPIO_PIN_0

void send_to_lcd (char data, int rs){
    HAL_GPIO_WritePin(RS_PORT, RS_PIN, rs);  // rs = 1 for data, rs=0 for command
    /* write the data to the respective pin */
    HAL_GPIO_WritePin(D7_PORT, D7_PIN, ((data>>3)&0x01));
    HAL_GPIO_WritePin(D6_PORT, D6_PIN, ((data>>2)&0x01));
    HAL_GPIO_WritePin(D5_PORT, D5_PIN, ((data>>1)&0x01));
    HAL_GPIO_WritePin(D4_PORT, D4_PIN, ((data>>0)&0x01));
    /* Toggle EN PIN to send the data
     * if the HCLK > 100 MHz, use the  20 us delay
     * if the LCD still doesn't work, increase the delay to 50, 80 or 100..
     */
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, 1);
    HAL_Delay(20);
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, 0);
    HAL_Delay(20);
}

void LCD_send_data (char data){
    char datatosend;

    /* send higher nibble */
    datatosend = ((data>>4)&0x0f);
    send_to_lcd(datatosend, 1);  // rs =1 for sending data
    /* send Lower nibble */
    datatosend = ((data)&0x0f);
    send_to_lcd(datatosend, 1);
}

void LCD_send_cmd (char cmd){
    char datatosend;

    /* send upper nibble first */
    datatosend = ((cmd>>4)&0x0f);
    send_to_lcd(datatosend,0);  // RS must be while sending command
    /* send Lower Nibble */
    datatosend = ((cmd)&0x0f);
    send_to_lcd(datatosend, 0);
}

void LCD_init(){
	HAL_Delay(50);
	LCD_send_cmd(0x03);
	HAL_Delay(5);
	LCD_send_cmd(0x03);
	HAL_Delay(1);
	LCD_send_cmd(0x03);
	HAL_Delay(10);
	LCD_send_cmd(0x02);
	HAL_Delay(10);

	LCD_send_cmd(0x28);
	HAL_Delay(1);
	LCD_send_cmd(0x08);
	HAL_Delay(1);
	LCD_send_cmd(0x01);
	HAL_Delay(1);
	LCD_send_cmd(0x06);
	HAL_Delay(1);
	LCD_send_cmd(0x0C);
}

void LCD_put_cur(int row, int col){
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xc0;
            break;
    }
    LCD_send_cmd (col);
}

void LCD_send_string(char *str){
	while (*str) {
			LCD_send_data(*str++);
		}
}

void LCD_clear(){
	LCD_send_cmd(0x01);
	HAL_Delay(2);
}

uint32_t LCD_button(ADC_HandleTypeDef *hadc1){
	HAL_ADC_Start(hadc1);
	HAL_ADC_PollForConversion(hadc1,100);
	uint32_t raw = HAL_ADC_GetValue(hadc1);
	return raw;
}

void LCD_print_float(float x){
	char uart_buf[50];
	sprintf(uart_buf, "%f        ", x);
	LCD_send_string(uart_buf);
}

void serial_print(char *str, UART_HandleTypeDef* huart2){
	char uart_buf[50];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, str);
	HAL_UART_Transmit(huart2, (unsigned char *) uart_buf, uart_buf_len, 100);
}
