/*
 * myheader.h
 *
 *  Created on: Nov 7, 2022
 *      Author: User
 */

#ifndef SRC_LCD_DISPLAY_H_
#define SRC_LCD_DISPLAY_H_

void LCD_clear();
void send_to_lcd (char data, int rs);
void LCD_send_data (char data);
void LCD_send_cmd (char cmd);
void LCD_init();
void LCD_put_cur(int row, int col);
void LCD_send_string(char*);

#endif /* SRC_MYHEADER_H_ */
