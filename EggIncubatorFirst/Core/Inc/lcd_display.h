/*
 * lcd_display.h
 *
 *  Created on: Aug 13, 2024
 *      Author: sam
 */

#ifndef SRC_LCD_DISPLAY_H_
#define SRC_LCD_DISPLAY_H_

#include "main.h"


void setDigit(uint8_t row, uint8_t digit, int decimalPoint);
void displayNumber(unsigned int number);
void lcd_init(SPI_HandleTypeDef * hspi);
void displayTemperature(int temperature);




#endif /* SRC_LCD_DISPLAY_H_ */
