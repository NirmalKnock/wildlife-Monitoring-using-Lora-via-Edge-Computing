/*
 * spi_main.c
 *
 *  Created on: Feb 15, 2021
 *      Author: Nirmal Kumar
 */

#include <string.h>
#include "stm32f446xx.h"

//Commands
#define COMMAND_LED_CRTL					0x50
#define COMMAND_SENSOR_READ			0X51
#define COMMAND_LED_READ					0x52
#define COMMAND_PRINT						0x53
#define COMMAND_ID_READ					0x54

#define LED_ON											1
#define LED_OFF										0

//Arduino Analog Pins


#define ANALOG_PIN0								0
#define ANALOG_PIN1								1
#define ANALOG_PIN2								2
#define ANALOG_PIN3								3
#define ANALOG_PIN4								4


//Arduino LED PIN

#define ARDUINO_LED_PIN					    9




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
	gpio_spi_config.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_14;
	GPIO_Init(&gpio_spi_config);

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



int SPI_VerfiyResponce(uint8_t ackbyte)
{
	if (ackbyte == 0xF5)
	{
		//then it is ACK
		return 1;
	}
		//It is NACK
		return 0;

}

uint8_t ackbyte;
uint8_t args[2];
uint8_t dummyWrite = 0xff;
uint8_t dummyRead;

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

				/*
				//Send the length information to slave
				uint8_t dataLen = strlen(user_Data);
				SPI_SendData(SPI2, &dataLen, 1);

				 //calling send data function
				 SPI_SendData(SPI2, (uint8_t*)user_Data, strlen(user_Data));
				 */

				//Commands
				// 1.  CMD_LED_CTRL   <PinNo>  <Value>

             /****************
				uint8_t commandcode = COMMAND_LED_CRTL;
				SPI_SendData(SPI2, &commandcode, 1);

				//But the slave send the data,back , that makes a flag SET RXNE
				//so clear off RXNE
				SPI_ReceiveData(SPI2, &dummyRead, 1);

				//Send dummy bit(1byte) ,to get ACK or NACK
				SPI_SendData(SPI2, &dummyWrite, 1);


				//SPI read data to get ACK or NACK
				SPI_ReceiveData(SPI2, &ackbyte, 1);

				//Creating a small function to verify ACK or NACK

				if (SPI_VerfiyResponce(ackbyte) == 1)
				{
					//Then only send the arguments
					args[0] = ARDUINO_LED_PIN;
					args[1] = LED_ON;
					SPI_SendData(SPI2, args, 1);
				}
				SPI_ReceiveData(SPI2,&dummyRead, 1);
				delay();
				*/

				// 2.Command Sensor Read   CMDSENSOR_READ   <analogpin(1)>
				uint8_t CommandAnalog0 = COMMAND_SENSOR_READ;
				uint8_t cmd2dummybyte;
				uint8_t cmd2dummywrite = 0xff;
				uint8_t cmd2ack;
				uint8_t cmd2args[2];
				SPI_SendData(SPI2,&CommandAnalog0, 1);

				//Recevie dummy byte
				SPI_ReceiveData(SPI2, &cmd2dummybyte, 1);

				//Insert some delay for the slave to respond ,because they work on ADC operation.
				delay();

				//Sending Dummybyte
				SPI_SendData(SPI2, &cmd2dummywrite, 1);

				//Receving ACK
				SPI_ReceiveData(SPI2, &cmd2ack, 1);

				if (SPI_VerfiyResponce(cmd2ack) ==1 )
				{
					    cmd2args[0] = ANALOG_PIN0;
						//Sending Command2
					    SPI_SendData(SPI2, cmd2args, 1);

						//**Read the Response**//
						//Receving Dummy byte to clear RXNE
						SPI_ReceiveData(SPI2, &cmd2dummybyte, 1);

						//Send the dummy byte
						SPI_SendData(SPI2, &cmd2dummywrite, 1);

						//Receiving the analog sensor data
						uint8_t analog_data;
						SPI_ReceiveData(SPI2, &analog_data, 1);
					}




				 //Get the status of BSY flag, wheather the TX buffer is empty or not

				 while ( SPI_Get_FlagStatus(SPI2, SPI_BSY_FLAG) );
				 //Disabling the SPI peripheral after transmitting all the data
				 SPI_PeripheralControl(SPI2, DISABLE);
	 }

	return 0;
}
