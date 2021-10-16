	/*
 * main_interpr.c
 *
 *  Created on: Jan 15, 2021
 *      Author: Nirmal Kumar
 */
#include <stm32f446xx.h>
#include <string.h>

#define BTN_PRESSED			1
#define BTN_NOT_PRESSED		0

void delay(){
	for (uint32_t i =0 ;i<2000 ;i++);
}

int main(void){
	//Before Inializing set Member elments is 0
	GPIO_Handle_t Gpioled , Gpiobtn ;
	memset(&Gpioled, 0 , sizeof(Gpioled) );     // Structure name , value , size
	memset(&Gpiobtn, 0 , sizeof(Gpiobtn) );

	//For Led

	Gpioled.pGPIOx	= GPIOA;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&Gpioled);



	//For Button

	Gpiobtn.pGPIOx = GPIOC;
	Gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	Gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_FT;
	Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed 	= GPIO_SPEED_FAST;
	Gpiobtn.GPIO_PinConfig.GPIO_PinOPType	= GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&Gpiobtn);


	//IRQ interrupt Config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);


}


void EXTI15_10_IRQHandler(void ){

	//1. Handle the interrupt, Pin Number
	delay();
	GPIO_IRQHandling(GPIO_PIN_NUMBER_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUMBER_5);

}
