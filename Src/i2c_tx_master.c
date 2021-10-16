/*
 * i2c_tx_master.c
 *
 *  Created on: May 26, 2021
 *      Author: Nirmal Kumar
 */

#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>

uint8_t SLAVE_ADDR=	 	0x68;
uint8_t SLAVE_ADDR1=	 	0x68;
#define MY_ADDR			0X61

//I2C_Structure Confuguration
I2C_Handle_t I2C1_Handle;

//Some data
uint8_t some_data[] = "Hi\n";
char data[] = "hi";



//I2C Configurations Pins

//PB6 - SCL
//PB7- SDA

/*
 * -------------------------------Function Prototypes-------------------------------------------------
 */
void I2C_Gpio_Init(void);
void I2C_Peri_Init(void);
void Gpio_Button(void);
void delay(void);


//delay
void delay(void)
{
	for (int i =0 ; i<500000;i++);
}


void I2C_Gpio_Init(void)
{
	GPIO_Handle_t 	I2C_gpio;
	//clear all bits in structures


	I2C_gpio.pGPIOx = GPIOB;
	I2C_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2C_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	I2C_gpio.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;

	//SCL
	I2C_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_6;
	GPIO_Init(&I2C_gpio);

	//SDA
	I2C_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_7;
	GPIO_Init(&I2C_gpio);

}


void I2C_Peri_Init(void)
{


	I2C1_Handle.pI2Cx = I2C1;
	//Clearning bits in structure



	I2C1_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; 				//Check I2C spec , some addresses  are reserved (not needed for Master Mode
	I2C1_Handle.I2C_Config.I2C_FMDuty_Cycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
				//Not needed for SM mode

	I2C_Init(&I2C1_Handle);

}

void Gpio_Button(void)
{
	GPIO_Handle_t Gpio_btn;
	Gpio_btn.pGPIOx  = GPIOC;

	//Clearning all bits in the structure
	Gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	Gpio_btn.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_IN;
	Gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&Gpio_btn);

}

int main(void)
{

	//I2C gpio Configuration
	I2C_Gpio_Init();


	//I2C peripheral Configuration
	I2C_Peri_Init();


	//GPIO_button configuration
	Gpio_Button();

	//I2C peripheral Control
	I2C_PeripheralControl(I2C1, ENABLE);
	I2C_ManagaeAcking(I2C1, I2C_ACK_ENABLE);

	while (1)
	{
		//wait for button Press
		while ( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUMBER_13) );
		delay();

		//Send Some data to slave
		I2C_MasterSendData(&I2C1_Handle, some_data, strlen((char *)some_data), SLAVE_ADDR,I2C_ENABLE_SR);
		delay();
		I2C_MasterSendData(&I2C1_Handle, some_data, strlen((char *)some_data), SLAVE_ADDR,I2C_DISABLE_SR);




	}






	return 0 ;
}
