/*
 * 017rtc_lcd.c
 *
 *  Created on: Jun 21, 2021
 *      Author: Nirmal Kumar
 */



#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#include <stdint.h>

extern void initialise_monitor_handles(void);
void number_to_string(uint8_t num, char* buf);
char* time_to_string(RTC_time_t *pRtc_time);
char* get_day_of_week(uint8_t i);
void init_systick_timer(uint32_t tick_hz);
char* date_to_string(RTC_date_t *pRtc_date);




#define SYSTICK_TIM_CLICK 	16000000UL
#define TICK_HZ					1			//one interrupt for evry 1 sec



int main(void)
{

	RTC_date_t current_date;
	RTC_time_t current_time;
	lcd_Init();

	Lcd_Print_String("RTC hello...");
	mdelay(500);
	Lcd_Display_clear();
	Lcd_Display_Return_to_Home();

	//DS1307 initializtion
	if (ds1307_Init())				//if CH =1, then hangs here
	{
		printf("failed");
		while (1);
	}


	//Calling systick init
	init_systick_timer(TICK_HZ);			//1 interrupt for every 1 sec, eg: 10 interrupt for evry 1 sec, so div by 10

	current_date.day = SATURDAY
	current_date.date = 27;
	current_date.month = 6;
	current_date.year = 21;


	//Configuring the time
	current_time.hours = 11;
	current_time.minute = 59;
	current_time.seconds = 50;
	current_time.time_format = TIME_FORMAT_12HRS_PM;


	//Set current date
	ds1307_set_current_date(&current_date);

	//set current time
	ds1307_set_current_time(&current_time);



	while(1);

	return 0;
}




