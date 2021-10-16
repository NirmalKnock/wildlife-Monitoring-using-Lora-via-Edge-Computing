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


#define LCD_TIME_SET	1				//set and clear as per the code


#define SYSTICK_TIM_CLICK 	16000000UL
#define TICK_HZ					1			//one interrupt for evry 1 sec



int main(void)
{

	RTC_date_t current_date;
	RTC_time_t current_time;
	lcd_Init();


	//DS1307 initializtion
	if (ds1307_Init())				//if CH =1, then hangs here
	{
		printf("failed");
		while (1);
	}



#ifndef LCD_TIME_SET

	Lcd_Print_String("RTC hello...");
	mdelay(500);
	Lcd_Display_clear();
	Lcd_Display_Return_to_Home();

	current_date.day = MONDAY
	current_date.date = 28;
	current_date.month = 6;
	current_date.year = 21;


	//Configuring the time
	current_time.hours = 8;
	current_time.minute = 29;
	current_time.seconds = 00;
	current_time.time_format = TIME_FORMAT_12HRS_PM;


	//Set current date
	ds1307_set_current_date(&current_date);

	//set current time
	ds1307_set_current_time(&current_time);

#else
	//Get current time
	ds1307_get_current_time(&current_time);
	//Get current data
	ds1307_get_current_date(&current_date);


#endif

	//Calling systick init
	init_systick_timer(TICK_HZ);			//1 interrupt for every 1 sec, eg: 10 interrupt for evry 1 sec, so div by 10

			char *am_pm = 0;
			if (current_time.time_format != TIME_FORMAT_24HRS)
			{
				am_pm = (current_time.time_format) 	? 	"PM" : "AM" ;
				//printf("Current time is = %s %s\n",time_to_string(&current_time),am_pm); 			//01:45:10
				LCD_Set_Cursor(1, 4);
				Lcd_Print_String(time_to_string(&current_time));
				LCD_Set_Cursor(1, 10);
				Lcd_Print_String(am_pm);





			}
			else	//24hr format
			{
				//printf("Current time is %s\n",time_to_string(&current_time));
				Lcd_Print_String(time_to_string(&current_time));
			}



			//Print current day
			//printf("Current day = %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));
			LCD_Set_Cursor(2, 3);
			//Get current data
			Lcd_Print_String(date_to_string(&current_date));

		//	Lcd_Print_String(get_day_of_week(current_date.day));




	while(1);

	return 0;
}



//systick handler
void SysTick_Handler(void)
{

		RTC_date_t current_date;
		RTC_time_t current_time;

		//Get current time
		ds1307_get_current_time(&current_time);


		LCD_Set_Cursor(1, 4);
		char *am_pm;
		if (current_time.time_format != TIME_FORMAT_24HRS)
		{
			am_pm = (current_time.time_format) 	? 	"PM" : "AM" ;
			Lcd_Print_String(time_to_string(&current_time));
			LCD_Set_Cursor(1, 13);
			Lcd_Print_String(am_pm);
			//printf("%s",time_to_string(&current_time));
		}
		else	//24hr format
		{
			printf("Current time is %s\n",time_to_string(&current_time));
		}




		//Get current data
		ds1307_get_current_date(&current_date);
		LCD_Set_Cursor(2, 3);

		//Print current day
		//printf("Current day = %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));

		 Lcd_Print_String(date_to_string(&current_date));
		 LCD_Set_Cursor(2, 12);
		 Lcd_Print_char('(');
		 LCD_Set_Cursor(2, 13);
		 Lcd_Print_String(get_day_of_week(current_date.day));
		 LCD_Set_Cursor(2, 16);
		 Lcd_Print_char(')');


}





//helper function
void number_to_string(uint8_t num, char* buf)
{
	if (num < 10)
	{
		buf[0] = '0';
		buf[1] = (num + 48) ; 		//convert ascii equivalent
	}
	else if (num >= 10 && num < 99)
	{
		buf[0] = ((num /10) + 48);
		buf[1] = ((num % 10) + 48);
	}

}

//hh:mm:ss (time to string)conversion
char* time_to_string(RTC_time_t *pRtc_time)
{
	static char buf[9]; // because total 9 bits

	buf[2] = ':';
	buf[5] = ':';

	number_to_string (pRtc_time->hours,buf);
	number_to_string (pRtc_time->minute,&buf[3]);
	number_to_string (pRtc_time->seconds,&buf[6]);

	buf[8] = '\0';

	return buf;
}


/*
 * Get the day of week
 */
char* get_day_of_week(uint8_t i)
{
	char* day[] = {"SUN","MON","TUE", "WED","THU","FRI","SAT"};

	return day[i-1];
}


/*
 * Get date to string
 */

//dd/mm/yy
char* date_to_string(RTC_date_t *pRtc_date)
{
	static char buf[9];

	buf[2] = '/';
	buf[5] = '/';

	number_to_string(pRtc_date->date, buf);
	number_to_string(pRtc_date->month, &buf[3]);
	number_to_string(pRtc_date->year, &buf[6]);

	buf[8] = '\0';

	return buf;

}

/*
 * Systick interrupt generation for every 1 sec. to get time and date
 */

void init_systick_timer(uint32_t tick_hz)
{
	//	Configuring the base address of systick register
	uint32_t *pSTRVR = (uint32_t*)0xE000E014;			//Reload value register
	uint32_t *pSTCSR = (uint32_t*)0xE000E010;			//Control and status register

	//Calculation of Reload Vaue
	uint32_t count_value = (( SYSTICK_TIM_CLICK / tick_hz) - 1);			//1 interrupt for evry 1 sec

	//Clear the value for Reload value register
	*pSTRVR &= ~(0x00FFFFFF);

	//Load the the value to Reload value register
	*pSTRVR |= (count_value);

	//Configuring the control and status register
	*pSTCSR |=  ( 1 << 1);			//TICKINT bit to generate interrrupt

	*pSTCSR |= ( 1 << 2 ); 			//To set the clock source as processor clock

	*pSTCSR |= ( 1 << 0 );			//Enable the systick Timer
}



