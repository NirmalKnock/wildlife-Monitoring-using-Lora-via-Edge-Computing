/*
 * uart_tx.c
 *
 *  Created on: Jun 13, 2021
 *      Author: Nirmal Kumar
 */

#include "stm32f446xx.h"
#include <string.h>
#include<stdio.h>
void GPIO_Uart(void);
void Button(void);
void USART_Config(void);
void IRQ_Enable();
//uint32_t USART_STD_BAUD_115200 = 115200;

USART_Handle_t Usart;



//we have 3 different messages that we transmit to arduino
//you can by all means add more messages
char *msg[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};

//reply from arduino will be stored here
char rx_buf[50];


//This flag indicates reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data = 0;



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
	Usart.pUSART_Config.USART_Mode = USART_MODE_TXRX;
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

void IRQ_Enable()
{
	USART_IRQInterruptConfig(IRQ_NO_USART1, ENABLE);
}







int main(void)
{

	uint32_t cnt = 0;
	GPIO_Uart();

	USART_Config();

	IRQ_Enable();

	USART_PeripheralControl(USART1, ENABLE);




	Button();

	while ( 1)
	{

	while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUMBER_13));
	delay();

	// Next message index ; make sure that cnt value doesn't cross 2
			cnt = cnt % 3;

			//First lets enable the reception in interrupt mode
			//this code enables the receive interrupt
			//	while ( USART_ReceiveDataIT(&Usart,(uint8_t*)rx_buf,strlen(msg[cnt])) != USART_READY );

			//Send the msg indexed by cnt in blocking mode
			//	USART_SendData(&Usart,(uint8_t*)msg[cnt],strlen(msg[cnt]));

			while (USART_ReceiveDataIT(&Usart, (uint8_t*)rx_buf, strlen(msg[cnt])) != I2C_READY);
			delay();
			while ((USART_SendDataIT(&Usart, (uint8_t*)msg[cnt],strlen(msg[cnt]))) != I2C_READY);




			//while (USART_SendDataIT(&Usart, (uint8_t*)rx_buf, sizeof(rx_buf)));




	    	//Now lets wait until all the bytes are received from the arduino .
	    	//When all the bytes are received rxCmplt will be SET in application callback
	    	while(rxCmplt != SET);
	    	while (USART_SendDataIT(&Usart, (uint8_t*)rx_buf, sizeof(rx_buf)));


	    	//just make sure that last byte should be null otherwise %s fails while printing
	    	rx_buf[strlen(msg[cnt])+ 1] = '\0';



	    	//invalidate the flag
	    	rxCmplt = RESET;

	    	//move on to next message indexed in msg[]
	    	cnt ++;
	    }

	return 0;
}

void USART1_IRQHandler(void)

{
	USART_IRQHandling(&Usart);
}

//Application call back
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
	if ( AppEv == USART_EVENT_TX_CMPLT)
	{
		;
	}else if ( AppEv == USART_EVENT_RX_CMPLT)
	{
		rxCmplt = SET;


	}
	}
