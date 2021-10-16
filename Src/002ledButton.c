/*
 * 002ledButton.c
 *
 *  Created on: Jan 7, 2021
 *      Author: Nirmal Kumar
 */


#include <stm32f446xx.h>

#define BTN_PRESSED			1
#define BTN_NOT_PRESSED		0


void delay(){

	for(uint32_t i=0;i<=100000;i++);
}

int main(void){

	//Initialize the GPIO for led

	GPIO_Handle_t Gpioled , Gpiobtn;
	GPIO_PeriClockControl(GPIOA, ENABLE);

	Gpioled.pGPIOx														= GPIOA;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber				= GPIO_PIN_NUMBER_5;
	Gpioled.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_NO_PUPD;

	GPIO_Init(&Gpioled);


	//Initialize the GPIO of Button
	GPIO_PeriClockControl(GPIOC, ENABLE);

	Gpiobtn.pGPIOx										= GPIOC;
	Gpiobtn.GPIO_PinConfig.GPIO_PinNumber				= GPIO_PIN_NUMBER_13;
	Gpiobtn.GPIO_PinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_SPEED_FAST;
	Gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl			= GPIO_NO_PUPD;

	GPIO_Init(&Gpiobtn);



	while(1){

		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUMBER_13) == !BTN_PRESSED){

			delay();
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUMBER_5, ENABLE);
		}
		else{
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUMBER_5, DISABLE);

		}

	}




	return 0;
}
