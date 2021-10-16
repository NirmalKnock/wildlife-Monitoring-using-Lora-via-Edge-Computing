/*
 * ds1307.h
 *
 *  Created on: Jun 21, 2021
 *      Author: Nirmal Kumar
 */

#ifndef DS1307_H_
#define DS1307_H_

#include "stm32f446xx.h"




/*
 * Application configurable Items
 */
#define DS1307_I2C					I2C1					//I2C1 selected
#define DS1307_I2C_GPIO_PORT		GPIOB					//AT PORT B
#define DS1307_I2C_SDA_PIN			GPIO_PIN_NUMBER_7		//SDA
#define DS1307_I2C_SCL_PIN			GPIO_PIN_NUMBER_6		//SCL
#define DS1307_I2C_SPEED			I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD				GPIO_PIN_PU				// internal Pullups



/*
 * DS1307 register definition MACROS
 */
#define DS1307_ADDR_SEC				0x00
#define DS1307_ADDR_MIN				0x01
#define DS1307_ADDR_HOURS			0x02
#define DS1307_ADDR_DAY				0x03
#define DS1307_ADDR_DATE			0x04
#define DS1307_ADDR_MONTH			0x05
#define DS1307_ADDR_YEAR			0x06


//Time in PM ,AM, 12 or 24HRS

#define TIME_FORMAT_12HRS_AM				0
#define TIME_FORMAT_12HRS_PM				1
#define TIME_FORMAT_24HRS					2

//Device slave address
#define DS1307_I2C_ADDRESS					0x68


//Day information
#define SUNDAY								1;
#define MONDAY								2;
#define TUESDAY								3;
#define WEDNESDAY							4;
#define THURSDAY							5;
#define FRIDAY								6;
#define SATURDAY							7;


/*
 * Creating a structure to hold the data
 *
 */

typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;

}RTC_date_t;


/*
 * Creating a sturcture to hold time information
 */

typedef struct
{
	uint8_t hours;
	uint8_t minute;
	uint8_t seconds;
	uint8_t time_format;
}RTC_time_t;


/*
 * Function prototypes
 */

uint8_t 	ds1307_Init(void);
void ds1307_set_current_time(RTC_time_t *pRTC_Time);
void ds1307_get_current_time(RTC_time_t *pRTC_Time);

void ds1307_set_current_date(RTC_date_t *pRTC_Date);
void ds1307_get_current_date(RTC_date_t *pRTC_Date);


#endif /* DS1307_H_ */
