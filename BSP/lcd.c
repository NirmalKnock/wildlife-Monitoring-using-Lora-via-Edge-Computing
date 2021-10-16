/*
 * lcd.c
 *
 *  Created on: Jun 21, 2021
 *      Author: Nirmal Kumar
 */

#include "lcd.h"
#include <string.h>



static void write_4_bits(uint8_t value);
static void Lcd_Enable(void);
void Lcd_Display_clear(void);



/*
 ****************************| LCD Inits|********************************************
 */
void lcd_Init(void)
{

	//1. Configure the gpio pins that we are used in the lcd Connections

	GPIO_Handle_t LCD_Gpio;

	memset(&LCD_Gpio,0,sizeof(LCD_Gpio));

	LCD_Gpio.pGPIOx = LCD_GPIO_PORT;			//GPIOB
	LCD_Gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LCD_Gpio.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	LCD_Gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LCD_Gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	LCD_Gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&LCD_Gpio);

	LCD_Gpio.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&LCD_Gpio);

	LCD_Gpio.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&LCD_Gpio);

	LCD_Gpio.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&LCD_Gpio);

	LCD_Gpio.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&LCD_Gpio);

	LCD_Gpio.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&LCD_Gpio);

	LCD_Gpio.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&LCD_Gpio);

	//Keep all value 0 , on the pin
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2. Configure the LCD initialization
	//Refer data sheet

	mdelay(40);

	/*Rs = 0, for LCD command Mode*/
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/*RW = 0. for LCD mode Write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//writing 0, 1 to lcd pin(helper function) , refer RM
	write_4_bits(0x3);					// 0 0 1 1			MSB - LSB

	mdelay(5);

	//writing 0, 1 to lcd pin(helper function) , refer RM
	write_4_bits(0x03);

	udelay(150);

	//writing 0, 1 to lcd pin(helper function) , refer RM
	write_4_bits(0x3);

	//writing 0, 1 to lcd pin(helper function) , refer RM
	write_4_bits(0x2);


	/*****command config*******/

	//Function set Command(refer RM ti send , which command first)
	//send the function command
	lcd_Send_Command(LCD_CMD_4DL_2N_5X8);

	mdelay(5);

	//send the display on off command
	lcd_Send_Command(LCD_CMD_DON_CURON);
	mdelay(5);


	//send display clear command
	Lcd_Display_clear();
	mdelay(5);

	//Entry mode set command
	lcd_Send_Command(LCD_CMD_INCADD);
	mdelay(5);


}


/**
  *   Set Lcd to a specified location given by row and column information
  *   Row Number (1 to 2)
  *   Column Number (1 to 16) Assuming a 2 X 16 characters display
  */
void LCD_Set_Cursor(uint8_t row, uint8_t coloumn)
{
	coloumn--;

	switch(row)
	{
		case 1:
			/*Set cursor to 1 row address and based on coloumn index*/
			lcd_Send_Command((coloumn |= 0x80));		//refer RM in 11th page
			break;
		case 2 :
			lcd_Send_Command((coloumn |= 0xC0));
			break;
		default:
			break;
	}

}








 void mdelay(uint32_t cnt)
{
	for (uint32_t i = 0; i < (cnt * 1000); i++);
}

 void udelay(uint32_t cnt)
{
	for (uint32_t i = 0; i < (cnt * 1); i++);
}





/*****************************************************************************|
 * LCD Enable
 * High to low transition
 */

static void Lcd_Enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN,GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN,GPIO_PIN_RESET);
	udelay(100);	//Execution time greater than 37 us
}


/******************************************************************************************
 * Writes 4 bits of command to LCD,ON pins D4, D5, D6, D7
 * // value = 0 0 1 1			MSB - LSB
 */

static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1));	//for D4 =1

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1));	//for D5 =1

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1));	//for D6 =0

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1));	//for D7 =0


	//to latch every thing
	Lcd_Enable();

}

/*
 * *********************************|LCD Send Command |************************************
 * The fucntion which sends the command to LCD
 * RS= 0 , Command Mode
 * WR= 0, Write mode
 */

 void lcd_Send_Command(uint8_t cmd)
{
	/*RS = 0, for lcd Command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);


	/* WR = 0, for write mode */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//Send the higher 4 bits
	write_4_bits(cmd >> 4);

	//Send the lower nibble
	write_4_bits(cmd & 0x0F);

}


/*
 * ********************************|LCD Send data |******************************************|
 *
 * This fucntion sends the data to LCD
 * First sends higher nibble
 * Second sends the Lower Nibble
 * During on High-to-low transition only
 * pins used D4,D5,D6,D7
 *
 * (Sending framing)
 * Higher nibble -  1   1   1   1
 * 					D7  D5  D6  D4
 * Lower nibble	 -  1	0	0	1
 * 					D7	D6	D5	D4
 */
 void Lcd_Print_char(uint8_t data)
 {
	 //1. Set the RS = 1 ( Data mode)
	 GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	 //2. Set WR = 0 , for (Write Mode)
	 GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);


	 //Send higher nibble
	 write_4_bits(data >> 4);

	 //Send Lower Nibble
	 write_4_bits(data & 0x0F);

 }

 /*
  * Helper function
  * ***********************Lcd_Display_clear********************
  */
void Lcd_Display_clear(void)
{
	//send the display cleaer commanf
	lcd_Send_Command(LCD_CMD_DIS_CLR);
	mdelay(2);

}


/*
 * *******************LCD Prnint String***********************************
 */

void Lcd_Print_String(char *message)
{

	do
	{
		Lcd_Print_char((uint8_t)*message++);

		mdelay(5);

	}
	while (*message != '\0');

}

/*
 * ********************LCD Return to Home**********************************
 */

void Lcd_Display_Return_to_Home(void)
{
	lcd_Send_Command(LCD_CMD_DIS_RETURN_HM);
	/*
	 * refer the RM of 24th page no
	 * it requires of some delay
	 */
	mdelay(2);
}


