/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Jun 13, 2021
 *      Author: Nirmal Kumar
 */

#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx.h"


//AHB Prescalar values
uint16_t  AHB_Prescalar[9]= {2,4,8,16,32,64,128,256,512};
//APB1 Prescalar values
uint16_t APB1_Prescalar[4] = {2,4,8,16};
uint16_t APB2_Prescalar[4] = {2,4,8,16};



//Creating a fucntion to get the value of PLL

uint32_t RCC_GetPLLOutputClk(void)
{
	return 0;
}


//Function to calculate the value of FREQ on APB1 bus (pclk1)

//1. identify the clock source
//2. Identify the AHB prescalar value
//3. Identify the APB1 prescalar value

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1;
	uint8_t clksrc, temp, ahbp,apb1p;
	uint32_t systemClk;

	//1. Identify the clock source
	clksrc = ((RCC->CFGR >> 2 ) & 0x3);

	if (clksrc ==0)
	{
		systemClk = 16000000;
	}
	else if (clksrc == 1 )
	{
		systemClk = 8000000;
	}
	else if (clksrc == 2 )
	{
		systemClk = RCC_GetPLLOutputClk();
	}

	//2.Identify the AHP Prescalar value
	//Read the value of HPRE in clock Configuration Register

	temp  = ((RCC->CFGR >> 4 ) & 0xF);

	if (temp < 8 )
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescalar[temp - 8 ]; // To get the first value from array
	}

	//3. Identify the APB PreScaler

	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp <4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB2_Prescalar[temp - 4];
	}

	//Finally getting the PCLK1 -> verify in clock diagram
	pclk1 = (systemClk / ahbp) / apb1p;

	return pclk1;
}




//To calculate peripheral clock on ABP2 bus

//1. identify the clock source
//2. Identify the AHB prescalar value
//3. Identify the APB2 prescalar value

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t Fpclk,clksrc, SystemClk, temp, ahbp,apb2;

	//1. Identify the clock source on CFGR register in RCC
			clksrc = ( RCC->CFGR >>1 ) & 0x3;

			if (clksrc == 0)
			{
				SystemClk = 16000000;
			}
			else if ( clksrc == 1 )
			{
				SystemClk = 80000000;
			}
			else if (clksrc == 2 )
			{
				SystemClk = RCC_GetPLLOutputClk();
			}

	//2.	Identify the prescalar in AHB IN HPRE BIT

			temp = ((RCC->CFGR >> 4) & 0xF);

			if ( temp < 8 )
			{
				ahbp = 1;
			}
			else if (temp >8)
			{
				ahbp = AHB_Prescalar[temp - 8];
			}

	//3. 	Identify the APB2 prescalar in APB2 bus , in PPRE1 BIT

			temp = ((RCC->CFGR >>10 ) & 0x7);

			if ( temp <4)
			{
				apb2 = 1;
			}
			else if ( temp >4)
			{
				apb2 = APB1_Prescalar[temp - 4];
			}


//Calculating the Peripheral clock frequency
			Fpclk = (SystemClk / ahbp ) 	/ 	apb2;

return Fpclk;

}


