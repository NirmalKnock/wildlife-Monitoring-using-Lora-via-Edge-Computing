/*
 * ds1307.c
 *
 *  Created on: Jun 21, 2021
 *      Author: Nirmal Kumar
 */


#include "ds1307.h"
#include <string.h>
#include <stdint.h>

//function prototypes
static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_peripheral_config(void);
static void ds1307_Write(uint8_t value, uint8_t word_address);
static uint8_t ds1307_Read(uint8_t word_address);
static uint8_t binary_to_bcd(uint8_t word_value);
static uint8_t  bcd_to_binary(uint8_t word_value);


I2C_Handle_t g_ds1307I2cHandle;

#define  LCD_TIME_SET 1;			//set and clear as per the code


/************************************************************************************************
 *
  * @fn					- ds1307_Initl
  *
  * @brief				- Which initialize the DS1307 module
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or Disable value(MACROS)
  *	@param[in]			-
  *
  *	return				- uint8_t
  *
  *	@Note				- If the function returns 1 , that means CH =1, RTC is not work (init failed)
  *							- If the function return 0, that means CH =0, RTC is work 	(inits passed)

*/

uint8_t ds1307_Init(void)
{
		// 1. Init the I2C pins
		ds1307_i2c_pin_config();

		//2.	Initialize the peripheral
		ds1307_i2c_peripheral_config();

		//3. 	Enalbing the I2C peripheral
		I2C_PeripheralControl(DS1307_I2C, ENABLE);
#ifndef LCD_TIME_SET
		//4.	Make the CH bits as 0 to , start the RTC (CLOCK halt)
		ds1307_Write(0x00,DS1307_ADDR_SEC);				//Value , word section

#endif
		//5.	Read back the CH bit, to check if it set or not
		uint8_t Clock_State = ds1307_Read(DS1307_I2C_ADDRESS);



	return ( (Clock_State >> 7 ) & 0x1);


}

/*		__under progress__

uint8_t DS1307_Deint(void)
{
	ds1307_Write((binary_to_bcd(80)),DS1307_ADDR_SEC);
	uint8_t Clock_State = ds1307_Read(DS1307_I2C_ADDRESS);

	return ((Clock_State >> 7 ) & 0x1);
}
*/



/************************************************************************************************
 *
  * @fn					- dds1307_set_current_time
  *
  * @brief				- Which sets the current time
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void ds1307_set_current_time(RTC_time_t *pRTC_Time)
{

			uint8_t seconds, hrs;
			seconds = binary_to_bcd (pRTC_Time->seconds);
			seconds &= ~(1 << 7);  //CH set to 0
			//sending value to RTC
			ds1307_Write(seconds, DS1307_ADDR_SEC);

			//Sending minutes
			ds1307_Write(binary_to_bcd(pRTC_Time->minute), DS1307_ADDR_MIN);

			//Sending Hours
			hrs = binary_to_bcd(pRTC_Time->hours);

			if (pRTC_Time->time_format == TIME_FORMAT_24HRS)
			{
				//Set the 6bits as 0, for 24hr mode
				hrs &= ~ ( 1 << 6);
			}
			else
			{
				//sET THE 6 BIT as 1 for 12hr mode
				hrs |=  ( 1 << 6);
				hrs = (pRTC_Time->time_format == TIME_FORMAT_12HRS_PM) ?  hrs | ( 1 << 5)  : 	hrs & ~ ( 1 << 5) ;
			}

			//Sending hours
			ds1307_Write(hrs , DS1307_ADDR_HOURS);

}


/************************************************************************************************
 *
  * @fn					- ds1307_set_current_date
  *
  * @brief				- Which sets the current time
  *
  *	@param[in]			-  Address of RTC structure
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void ds1307_set_current_date(RTC_date_t *pRTC_Date)
{
	//Set current data
	ds1307_Write((binary_to_bcd(pRTC_Date->date)), DS1307_ADDR_DATE);
	//Set current month
	ds1307_Write((binary_to_bcd(pRTC_Date->month)), DS1307_ADDR_MONTH);
	//set current Year
	ds1307_Write((binary_to_bcd(pRTC_Date->year)), DS1307_ADDR_YEAR);
	//set day
	ds1307_Write(binary_to_bcd(pRTC_Date->day), DS1307_ADDR_DAY);


}

/************************************************************************************************
 *
  * @fn					- ds1307_get_current_time
  *
  * @brief				- Which Gets the current time from the RTC
  *
  *	@param[in]			-  Address of RTC structure
  *	@param[in]			-  )
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void ds1307_get_current_time(RTC_time_t *pRTC_Time)
{
	uint8_t seconds , hrs;
	seconds = ds1307_Read(DS1307_ADDR_SEC);
	//Clear the CH bit, we don;t require
	seconds &= ~ ( 1 << 7);

	//After that convert BCD to Binary, and updating in structure
	pRTC_Time->seconds = bcd_to_binary(seconds);
	pRTC_Time->minute = bcd_to_binary(ds1307_Read(DS1307_ADDR_MIN));

	//getting the hrs
	hrs = ds1307_Read(DS1307_ADDR_HOURS);

		//--To get status of AM or PM and Time format--//

			//Get the time format
			if (hrs & (1 << 6))
			{
						//12 hrs mode is selected
						pRTC_Time->time_format	 =	! ((hrs & (1 << 5)) == 0	);
						//Clearning 5 th and 6th positions, we don;t want this
						hrs &= ~ ( 0x3 << 5);
			}
			else
			{
				//24hrs mode is selected
				pRTC_Time->time_format = TIME_FORMAT_24HRS;
			}

	//storing a hours
	pRTC_Time->hours = bcd_to_binary(hrs);

}


/************************************************************************************************
 *
  * @fn					- ds1307_get_current_date
  *
  * @brief				- Which gets the data from RTC
  *
  *	@param[in]			-  Address of Rtc data  base address
  *	@param[in]			-  Enable or Disable value(MACROS)
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void ds1307_get_current_date(RTC_date_t *pRTC_Date)
{

	pRTC_Date->date			= bcd_to_binary(ds1307_Read(DS1307_ADDR_DATE));
	pRTC_Date->day 			= bcd_to_binary(ds1307_Read(DS1307_ADDR_DAY));
	pRTC_Date->month		= bcd_to_binary(ds1307_Read(DS1307_ADDR_MONTH));
	pRTC_Date->year 		= bcd_to_binary(ds1307_Read(DS1307_ADDR_YEAR));
}





static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	//for SDA
	//clear the structure
	memset(&i2c_sda,0, sizeof(i2c_sda));
	memset(&i2c_scl,  0, sizeof(i2c_scl));

	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_Init(&i2c_sda);

	//For SCL
	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_Init(&i2c_scl);
}


static void ds1307_i2c_peripheral_config(void)
{
	g_ds1307I2cHandle.pI2Cx = DS1307_I2C;
	g_ds1307I2cHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_ds1307I2cHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_Init(&g_ds1307I2cHandle);

}

static void ds1307_Write(uint8_t value, uint8_t word_address)
{
	uint8_t tx[2];
	tx[0] = word_address;
	tx[1] = value;

	I2C_MasterSendData(&g_ds1307I2cHandle, tx, 2 , DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

}


static uint8_t ds1307_Read(uint8_t word_address)				//first send device addres,and word address,, after to read,, refer RM for detials
{
	uint8_t read_value;
	I2C_MasterSendData(&g_ds1307I2cHandle, &word_address, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
	I2C_MasterReceiverData(&g_ds1307I2cHandle, &read_value, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

	return read_value;

}

static uint8_t binary_to_bcd(uint8_t word_value)
{
	uint8_t m_value, n_value;
	uint8_t bcd_value ;

	bcd_value = word_value;

	if( word_value >= 10)
	{
		m_value 	= word_value / 10;
		n_value	= word_value %10;
		bcd_value = (uint8_t)((m_value <<4) | (n_value) );
	}

	return bcd_value;
}

static uint8_t  bcd_to_binary(uint8_t word_value)
{
	uint8_t m_value, n_value;

	m_value = (uint8_t) (( word_value >> 4) * 10 );
	n_value = ((word_value & (uint8_t) 0x0F));			//mask bit



	return (m_value + n_value);
}

