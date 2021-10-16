/*
 * spi_main.c
 *
 *  Created on: Feb 15, 2021
 *      Author: Nirmal Kumar
 */

#include <string.h>
#include "stm32f446xx.h"

char user_Data[] ="kk";
#define GPIO_ALF_REGISTER    (GPIOB_BASEADDR + 0x00000020)

void  SPI2_GPIOInits(void)
{
	GPIO_Handle_t gpio_spi_config;
	gpio_spi_config.pGPIOx = GPIOB;
	gpio_spi_config.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	gpio_spi_config.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	gpio_spi_config.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_spi_config.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpio_spi_config.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//1.SCLK
	gpio_spi_config.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	GPIO_Init(&gpio_spi_config);

	// 2. MOSI
	gpio_spi_config.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_15;
	GPIO_Init(&gpio_spi_config);

	//3. MISO
	//gpio_spi_config.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_14;
	//GPIO_Init(&gpio_spi_config);

	//4. NSS
	gpio_spi_config.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_9;
	GPIO_Init(&gpio_spi_config);


}
void SPI2_Inits(void){

	    SPI_Handle_t SPI_Send;
		SPI_Send.pSPIx  = SPI2;
		SPI_Send.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
		SPI_Send.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
		SPI_Send.SPI_Config.SPI_Speed = SPI_SCLK_SPEED_DIV8; 			//	prescalar 8mhz
		SPI_Send.SPI_Config.SPI_DFF    = SPI_DFF_8BITS;
		SPI_Send.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
		SPI_Send.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
		SPI_Send.SPI_Config.SPI_SSM  = SPI_SSM_DI;								// Hardware slave management enabled

		SPI_Init(&SPI_Send);

}


void Button_Init()
{
	 GPIO_Handle_t Input_Button;
	 Input_Button.pGPIOx = GPIOC;
	 Input_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	 Input_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	 Input_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	 Input_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	 GPIO_Init(&Input_Button);
}

void delay(){
	for (uint32_t i =0 ;i<500000 ;i++);
}


int main(void){

	/* PB15 - SPI2_MOSI  - 11th pin Arduino
		PB14 - SPI2_MISO - 12th pin Arduino
		PB10  - SPI2_SCK -  13th pin 	Arduino
	 	PB9 - SPI2_NSS		- 10th pin arduino
	    Mode - alt 5     */


	// calling GPIOinits user function
	 SPI2_GPIOInits();


	 //calling SPIinits user function.
	 SPI2_Inits();

	 //Enabling SPI_SSOE control Register
	 SPI_SSOEConfig(SPI2, ENABLE);

	 //Enabling the SSI,this makes SSI siganl internally to High and Avoids MODEF error
	// SPI_SSIConfig(SPI2, ENABLE);x



      // Calling Button_Init user function
	 Button_Init();

	 while(1)
	 {

				 while ( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUMBER_13));

				 delay();

				 //enabling the SPI peripheral in control register
				SPI_PeripheralControl(SPI2, ENABLE);

				//Send the length information to slave
				uint8_t dataLen = strlen(user_Data);
				SPI_SendData(SPI2, &dataLen, 1);

				 //calling send data function
				 SPI_SendData(SPI2, (uint8_t*)user_Data, strlen(user_Data));






				 //Get the status of BSY flag, wheather the TX buffer is empty or not

				 while ( SPI_Get_FlagStatus(SPI2, SPI_BSY_FLAG) );
				 //Disabling the SPI peripheral after transmitting all the data
				 SPI_PeripheralControl(SPI2, DISABLE);
	 }

	return 0;
}
