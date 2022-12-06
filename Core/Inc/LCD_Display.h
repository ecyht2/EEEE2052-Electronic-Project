/*
 * myheader.h
 *
 *  Created on: Nov 7, 2022
 *      Author: User
 */

#ifndef SRC_MYHEADER_H_
#define SRC_MYHEADER_H_
#include "main.h"
#include <inttypes.h>

/**
 * @breif Enumeration of LCD Buttons.
 */
typedef enum LCDButtons {
	RIGHT, UP, DOWN, LEFT, SELECT
} LCDButtons;

void LCD_clear();
void send_to_lcd(char data, int rs);
void LCD_send_data(char data);
void LCD_send_cmd(char cmd);
void LCD_init();
void LCD_put_cur(int row, int col);
void LCD_send_string(char*);
LCDButtons LCD_get_pressed_button(ADC_HandleTypeDef *hadc);
void LCD_print_float(char *description, float x);

#endif /* SRC_MYHEADER_H_ */
