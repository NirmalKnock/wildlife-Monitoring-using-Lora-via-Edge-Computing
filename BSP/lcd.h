/*
 * lcd.h
 *
 *  Created on: Jun 21, 2021
 *      Author: Nirmal Kumar
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f446xx.h"

/*
 * BSP packages for LCD
 */

void lcd_Init(void);

void lcd_Send_Command(uint8_t cmd);

void Lcd_Print_char(uint8_t data);

void Lcd_Print_String(char *message);

void Lcd_Display_Return_to_Home(void);

void LCD_Set_Cursor(uint8_t row, uint8_t col);

void Lcd_Display_clear(void);

void Lcd_Print_char(uint8_t data);



void mdelay(uint32_t cnt);
void udelay(uint32_t cnt);
/*
 * Application configurable items
 */

#define LCD_GPIO_PORT		GPIOA						//GPIO PORT 	A
#define LCD_GPIO_RS			GPIO_PIN_NUMBER_0			//RS PIN 		0
#define LCD_GPIO_RW			GPIO_PIN_NUMBER_1			//RW PIN 		1
#define LCD_GPIO_EN			GPIO_PIN_NUMBER_4			//ENABLE PIN	4
#define LCD_GPIO_D4			GPIO_PIN_NUMBER_10			//D4 PIN 		10
#define LCD_GPIO_D5			GPIO_PIN_NUMBER_6			//D6 PIN		6
#define LCD_GPIO_D6			GPIO_PIN_NUMBER_7			//D7 PIN 		7
#define LCD_GPIO_D7			GPIO_PIN_NUMBER_8			//D8 PIN 		8

/*
 * LCD Commands
 */

#define LCD_CMD_4DL_2N_5X8		0x28		// 4data line, 2 line used , 5x8 pixel(Function set cmd)
#define LCD_CMD_DON_CURON		0x0E		// Display On , cursor On
#define LCD_CMD_INCADD       	0x06		// increment ram address
#define LCD_CMD_DIS_CLR			0x01		// display clear
#define LCD_CMD_DIS_RETURN_HM	0X02		// display return Home





#endif /* LCD_H_ */
