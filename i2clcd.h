#ifndef __I2CLCD_H
#define __I2CLCD_H

#include "stm32f1xx_hal.h"

void lcd_init (I2C_HandleTypeDef *hi2c);   // initialize lcd

void lcd_send_cmd (char cmd,I2C_HandleTypeDef *hi2c);  // send command to the lcd

void lcd_send_data (char data,I2C_HandleTypeDef *hi2c);  // send data to the lcd

void lcd_send_string (char *str,I2C_HandleTypeDef *hi2c);  // send string to the lcd

void lcd_clear_display (I2C_HandleTypeDef *hi2c);      //clear display lcd

void lcd_goto_XY (int row, int col,I2C_HandleTypeDef *hi2c); //set proper location on screen
#endif