/*
 * uart_tx.c
 *
 *  Created on: Jun 13, 2021
 *      Author: Nirmal Kumar
 */

#include "stm32f446xx.h"
#include <string.h>
void GPIO_Uart(void);
void Button(void);
void USART_Config(void);
//uint32_t USART_STD_BAUD_115200 = 115200;

USART_Handle_t Usart;
uint8_t data[]= {1,2,3,4,5,6,55};
char send_value = "s";
char send_data = "v";

uint8_t rcf_buffer[];
uint8_t rc_len;

//PA9 - Tx		D8
//PA10 - Rx	D2
void delay(void)
{
	for (int i =0 ; i<500000;i++);
}


void GPIO_Uart(void)
{
	//Configuring the GPIO

	GPIO_Handle_t usart_Gpio;

	usart_Gpio.pGPIOx = GPIOA;
	usart_Gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_Gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	usart_Gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_Gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_Gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Tx
	usart_Gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_9;
	GPIO_Init(&usart_Gpio);

	//Rx
	usart_Gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_10;
	GPIO_Init(&usart_Gpio);
}

void USART_Config(void)
{


	Usart.pUASRTx = USART1;
	Usart.pUSART_Config.USART_Mode = USART_MODE_ONLY_TX;
	Usart.pUSART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	Usart.pUSART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	Usart.pUSART_Config.USART_NoOfStopBits = USART_STARTBITS_1;
	Usart.pUSART_Config.USART_Baud = USART_STD_BAUD_115200;
	Usart.pUSART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&Usart);

}

void Button(void)
{

	GPIO_Handle_t btn;

	btn.pGPIOx = GPIOC;
	btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&btn);


}






int main(void)
{

	GPIO_Uart();

	USART_Config();

	USART_PeripheralControl(USART1, ENABLE);

	Button();
	while ( 1)
	{

	while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUMBER_13));
	delay();


	USART_SendData(&Usart, &send_data, strlen(send_data));
	USART_ReceiveData(&Usart, rcf_buffer, rc_len);

	USART_SendData(&Usart, &send_value,strlen((char*)send_value));
	USART_ReceiveData(&Usart, rcf_buffer, rc_len);



	}




	return 0;
}
