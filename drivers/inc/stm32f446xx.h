/*
 * ******************************************************************************
 * stm32f446xx.h																								|
 *																														|
 *  Created on: Nov 30, 2020																			|
 *      Author: Nirmal Kumar																				|
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 											|
 *******************************************************************************
*/

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#define __vo 				volatile
#define __weak 		 __attribute__((weak))
#include <stdint.h>
#include <stddef.h>


/************************************ Processor Specific Details******************************************************/
/*
 *  Arm Cortex Mx processor NVIC ISERx Register Address
 */

#define NVIC_ISER0 										((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 										((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 									    ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 										((__vo uint32_t*)0xE000E10C)

/*
 *  Arm Cortex Mx processor NVIC ICERx Register Address
 */

#define NVIC_ICER0 										((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1 										((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2 										((__vo  uint32_t*)0XE000E188)
#define NVIC_ICER3 										((__vo uint32_t*)0XE000E18C)

/*
 * Base Address of Priority Register
 */

#define NVIC_PR_BASE_ADDR							((__vo uint32_t *)0xE000E400)
#define No_PR_BITS_IMPLEMENTED								4 													// For STM MCU




#define NVIC_IRQ_PR0						0
#define NVIC_IRQ_PRI1						1
#define NVIC_IRQ_PRI2						2
#define NVIC_IRQ_PRI3						3
#define NVIC_IRQ_PRI4						4
#define NVIC_IRQ_PRI5						5
#define NVIC_IRQ_PRI6						6
#define NVIC_IRQ_PRI7						7
#define NVIC_IRQ_PRI8						8
#define NVIC_IRQ_PRI9						9
#define NVIC_IRQ_PRI10					10
#define NVIC_IRQ_PRI11					11
#define NVIC_IRQ_PRI12					12
#define NVIC_IRQ_PRI13					13
#define NVIC_IRQ_PRI14					14
#define NVIC_IRQ_PRI15					15













/*************************************************************************************************************************
 * Writing SRAM and FLASH base address
 * SRAM1 is base and we can update SRAM2 in Future
 */

#define FLASH_BASEADDR  					0x08000000U  						//Base address of Flash Memory
#define SRAM1_BASEADDR					0x20000000U							//Base address of SRAM1 Memory
#define SRAM											SRAM1_BASEADDR
#define SRAM2_BASEADDR					0x20001C00U							//Base address of SRAM2	Memory
#define ROM											0x1FFF0000U							//BASE address of ROM   Memory

/**************************************************************************************************************************
 * Writing AHBx and APBx peripheral Base address
 */

#define PERIPH_BASEADDR 				    0x40000000U 					     	//Base address of PERIPHERAL
#define APB1_PERIPH_BASEADDR	    	0x40000000U							//Base address of APB1 BUS
#define APB2_PERIPH_BASEADDR			0x40010000U							//Base address of APB2 BUS

#define AHB1_PERIPH_BASEADDR			0x40020000U							//Base address of AHB1 BUS
#define AHB2_PERIPH_BASEADDR			0x50000000U							//Base address of AHB2 BUS

/**************************************************************************************************************************
 * Writing all Peripherals base address
 */

#define GPIOA_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x0000)		//Base address of AHB1 BUS + GPIOA offset
#define GPIOB_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x0400)		//Base address of AHB1 BUS + GPIOB offset
#define GPIOC_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0800)		//Base address of AHB1 BUS + GPIOC offset
#define GPIOD_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0C00)		//Base address of AHB1 BUS + GPIOD offset
#define GPIOE_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x1000)		//Base address of AHB1 BUS + GPIOE offset
#define GPIOF_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x1400)		//Base address of AHB1 BUS + GPIOF offset
#define GPIOG_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1800)		//Base address of AHB1 BUS + GPIOG offset
#define GPIOH_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1C00)		//Base address of AHB1 BUS + GPIOH offset
#define RCC_BASEADDR						(AHB1_PERIPH_BASEADDR + 0x3800)		//Base address of AHB1 BUS + RCC offset


/**************************************************************************************************************************
 * Writing Base address of peripherals hanging on APB1 BUS
 */

#define I2C1_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5400)		//Base address of APB1 BUS + I2C1 offset
#define I2C2_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5800)		//Base address of APB1 BUS + I2C2 offset
#define I2C3_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5C00)		//Base address of APB1 BUS + I2C3 offset

#define SPI2_BASEADDR						(APB1_PERIPH_BASEADDR + 0x3800)		//Base address of APB1 BUS + SPI2 offset
#define SPI3_BASEADDR						(APB1_PERIPH_BASEADDR + 0x3C00)		//Base address of APB1 BUS + SPI3 offset

#define USART2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4400)		//Base address of APB1 BUS + USART2 offset
#define USART3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4800)		//Base address of APB1 BUS + USART3 offset
#define UART4_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4C00)		//Base address of APB1 BUS + UART4 offset
#define UART5_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5000)		//Base address of APB1 BUS + UART5 offset


/**************************************************************************************************************************
 * Writing Base address of peripherals hanging on APB2 BUS
 */
#define USART1_BASEADDR					(APB2_PERIPH_BASEADDR + 0x1000)		//Base address of APB2 BUS + UART1 offset
#define USART6_BASEADDR					(APB2_PERIPH_BASEADDR + 0x1400)		//Base address of APB2 BUS + UART6 offset

#define EXTI_BASEADDR    					(APB2_PERIPH_BASEADDR + 0x3C00)		//Base address of APB2 BUS + EXTI offset

#define SPI1_BASEADDR						(APB2_PERIPH_BASEADDR + 0x3000)		//Base address of APB2 BUS + SPI1 offset
#define SPI4_BASEADDR                	    (APB2_PERIPH_BASEADDR + 3400)			//Base address of APB2 BUS + SPI4 offset
#define SYSCFG_BASEADDR					(APB2_PERIPH_BASEADDR + 0x3800)		//Base address of APB2 BUS + SYSCFG offset

/***************************************************************************************************************************
 * Writing Peripheral Register Address using Struct
 * Due to volatile nature of registers we use Volatile
 */

typedef struct {

	__vo uint32_t MODER;													//GPIO port mode register
	__vo uint32_t OTYPER;													//GPIO port output type register
	__vo uint32_t OSPEEDER;												//GPIO port output speed register
	__vo uint32_t PUPDR;													//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;														//GPIO port input data register
	__vo uint32_t ODR;														//GPIO port output data register
	__vo uint32_t BSRR;														//GPIO port bit set/reset register
	__vo uint32_t LCKR;														//GPIO port configuration lock register
	__vo uint32_t AFR[2];													//GPIO alternate function low and High register

}GPIO_RegDef_t;


typedef struct {

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;


/***************************************************************************************************************************
 *Writing RCC Engine registers using Struct
 */

typedef struct {

	__vo uint32_t CR;														//Address of set 0x00
	__vo uint32_t PLLCFGR;													//Address of set 0x04
	__vo uint32_t CFGR;														//Address of set 0x08
	__vo uint32_t CIR;														//Address of set 0x0C
	__vo uint32_t AHB1RSTR;													//Address of set 0x10
	__vo uint32_t AHB2RSTR;													//Address of set 0x14
	__vo uint32_t AHB3RSTR;													//Address of set 0x18
	uint32_t	  RESERVED0;												//Address of set 0x1C
	__vo uint32_t APB1RSTR;													//Address of set 0x20
	__vo uint32_t APB2RSTR;													//Address of set 0x24
	uint32_t	  RESERVED1;												//Reserved1 : 0x28
	uint32_t	  RESERVED2;												//Reserved1 : 0x2c
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t	  RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t	  RESERVED4;
	uint32_t	  RESERVED5;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t	  RESERVED6;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t	  RESERVED7;
	uint32_t	  RESERVED8;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t	  RESERVED9;
	uint32_t	  RESERVED10;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

}RCC_Reg_Def_t;

/***************************************************************************************************************************
 * Writing Peripheral definitions for EXTI
 */

typedef struct{


	__vo uint32_t IMR;												//offset address 0x00
	__vo uint32_t EMR;												//offset address 0x04
	__vo uint32_t RTSR;												//offset address 0x08
	__vo uint32_t FTSR; 											//offset address 0x0C
	__vo uint32_t SWIER;											//offset address 0x10
	__vo uint32_t PR;												//offset address 0x14

}EXTI_RegDef_t;

/****************************************************************************************************
 * Peripheral Definitions fo SYSCFG Register
 */
typedef struct {

	__vo uint32_t MEMRMP;										//offset address 0x00
	__vo uint32_t PMC;												//offset address 0x04
	__vo uint32_t EXTICR[4];										//offset address 0x08-0x14
	__vo uint32_t CMPCR;											//offset address 0x20
	__vo uint32_t CFGR;												//offset address 0x2C

}SYSCFG_RegDef_t;

/******************************************************************************************************
 * Peripheral register definition structure for I2C
 */

typedef struct {

	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;


/****************************************************************************************************
 * Peripheral Register Definitions for USART Peripheral
 */

typedef struct
{
	__vo uint32_t USART_SR;
	__vo uint32_t USART_DR;
	__vo uint32_t USART_BRR;
	__vo uint32_t USART_CR1;
	__vo uint32_t USART_CR2;
	__vo uint32_t USART_CR3;
	__vo uint32_t USART_GTPR;

}USART_RegDef_t;







/***************************************************************************************************************************
 * Writing Peripheral definitions (Peripheral Base address (typecasted ) to GPIO_Reg_Def_t
 */

#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)		// Pointer access of GPIOA Baseaddress
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)		// Pointer access of GPIOB Baseaddress
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)		// Pointer access of GPIOC Baseaddress
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASEADDR)		// Pointer access of GPIOD Baseaddress
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASEADDR)		// Pointer access of GPIOE Baseaddress
#define GPIOF							((GPIO_RegDef_t*)GPIOF_BASEADDR)		// Pointer access of GPIOF Baseaddress
#define GPIOG							((GPIO_RegDef_t*)GPIOG_BASEADDR)		// Pointer access of GPIOG Baseaddress
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)		// Pointer access of GPIOH Baseaddress

#define RCC								((RCC_Reg_Def_t*)RCC_BASEADDR)			// Pointer access of RCC Engine struct


#define EXTI 							    ((EXTI_RegDef_t*)EXTI_BASEADDR)			// Pointer access of EXTI Baseaddress
#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)		// Pointer access of SYSCFG Baseaddress

/**************************************************************************************************************************************************
 * Writning Peripheral definitions (peripheral Base address (typecasted to ) to SPI_RegDef_t
 */

#define SPI1								((SPI_RegDef_t*)SPI1_BASEADDR)			// Pointer access of SPI1 Baseaddress
#define SPI2								((SPI_RegDef_t*)SPI2_BASEADDR)			// Pointer access of SPI2 Baseaddress
#define SPI3								((SPI_RegDef_t*)SPI3_BASEADDR)			// Pointer access of SPI3 Baseaddress
#define SPI4								((SPI_RegDef_t*)SPI4_BASEADDR)			// Pointer access of SPI3 Baseaddress

/***************************************************************************************************************************************************
 * Wrting Peripheral definitions to I2C peripheral (typecasting) to I2C_RegDef_t;
 */

#define I2C1								((I2C_RegDef_t*)I2C1_BASEADDR)			// Pointer access of I2C1 Baseaddres
#define I2C2								((I2C_RegDef_t*)I2C2_BASEADDR)			// Pointer access of I2C2 Baseaddres
#define I2C3								((I2C_RegDef_t*)I2C3_BASEADDR)			// Pointer access of I2C3 Baseaddres

/*****************************************************************************************************************************************************
 * Writing Peripheral Definitions for USART peripheral to USART_RegDef_t
 */
#define USART1							((USART_RegDef_t*)USART1_BASEADDR)
#define USART2							((USART_RegDef_t*)USART2_BASEADDR)
#define USART3							((USART_RegDef_t*)USART3_BASEADDR)
#define USART6							((USART_RegDef_t*)USART6_BASEADDR)

#define UART4							((USART_RegDef_t*)UART4_BASEADDR)
#define UART5							((USART_RegDef_t*)UART5_BASEADDR)



/****************************************************************************************************************************
 * Peripheral Clock Enable and disable Macros
 */
//Enable GPIOx Peripheral clocks

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1<<0))					//Enable GPIOA CLOCK
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1<<1))					//Enable GPIOB CLOCK
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1<<2))					//Enable GPIOC CLOCK
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1<<3))					//Enable GPIOD CLOCK
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1<<4))					//Enable GPIOE CLOCK
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1<<5))					//Enable GPIOF CLOCK
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1<<6))					//Enable GPIOG CLOCK
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1<<7))					//Enable GPIOH CLOCK


/*
 * Enable I2Cx peripheral register Clocks
 */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))					//Enable I2C1 CLOCK
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))					//Enable I2C2 CLOCK
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<23))					//Enable I2C3 CLOCK

/*
 * Enable SPIx Peripheral register clocks
 */
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))					//Enable SPI1 CLOCK
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))					//Enable SPI2 CLOCK
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))					//Enable SPI3 CLOCK
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1<<13))					//Enable SPI4 CLOCK

/*
 * Enable UARTx Peripheral register Clocks
 */

#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1<<19))					//Enable UART4 CLOCK
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1<<20))					//Enable UART5 CLOCK

/*
 * Enable USARTx Peripheral Register Clocks
 */

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<4))					//Enable USART1 CLOCK
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1<<17))					//Enable USART2 CLOCK
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1<<18))					//Enable USART3 CLOCK
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1<<5))					//Enable USART6 CLOCK

/*
 * SYSTEM CLOCK ENABLE
 */

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<14))					//Enable SYSTEM CLOCK

/*********************************************************************************************************************************
 * Disabling GPIOx Peripheral clocks
 */

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))					//Disabling GPIOA CLOCK
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<1))					//Disabling GPIOB CLOCK
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<2))					//Disabling GPIOC CLOCK
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<3))					//Disabling GPIOD CLOCK
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<4))					//Disabling GPIOE CLOCK
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<5))					//Disabling GPIOF CLOCK
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<6))					//Disabling GPIOG CLOCK
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<7))					//Disabling GPIOH CLOCK


/*
 * Disabling I2Cx peripheral register Clocks
 */

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<21))					//Disabling I2C1 CLOCK
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<22))					//Disabling I2C2 CLOCK
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<23))					//Disabling I2C3 CLOCK

/*
 * Disabling SPIx Peripheral register clocks
 */
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))					//Disabling SPI1 CLOCK
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1<<13))					//Disabling SPI4 CLOCK

#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))					//Disabling SPI2 CLOCK
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))					//Disabling SPI3 CLOCK


/*
 * Disable UARTx Peripheral register Clocks
 */

#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1<<19))					//Disabling UART4 CLOCK
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1<<20))					//Disabling UART5 CLOCK

/*
 * Disable USARTx Peripheral Register Clocks
 */

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<4))					//Disabling USART1 CLOCK
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<17))					//Disabling USART2 CLOCK
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<18))					//Disabling USART3 CLOCK
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1<<5))					//Disabling USART6 CLOCK

/*
 * SYSTEM CLOCK Disable
 */

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1<<14))					//Disable SYSTEM CLOCK



/**********************************************************************************************************************************************
 * Macros to RESET GPIOx peripherals
 **********************************************************************************************************************************************/

#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)	 // Set & Reset
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<1));  (RCC->AHB1RSTR &= ~(1<<1));}while(0)	 // Set & Reset
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<2));  (RCC->AHB1RSTR &= ~(1<<2));}while(0)	 // Set & Reset
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<3));  (RCC->AHB1RSTR &= ~(1<<3));}while(0)	 // Set & Reset
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<4));  (RCC->AHB1RSTR &= ~(1<<4));}while(0)	 // Set & Reset
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<5));  (RCC->AHB1RSTR &= ~(1<<5));}while(0)	 // Set & Reset
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<6));  (RCC->AHB1RSTR &= ~(1<<6));}while(0)	 // Set & Reset
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<7));  (RCC->AHB1RSTR &= ~(1<<7));}while(0)	 // Set & Reset

/************************************************************************************************************************************************
 * Macros to RESET SPIx Peripherals
 ************************************************************************************************************************************************/

#define SPI1_REG_RESET()				do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0) // Set & Reset
#define SPI2_REG_RESET()				do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0) // Set & Reset
#define SPI3_REG_RESET()				do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0) // Set & Reset
#define SPI4_REG_RESET()    			do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0) // Set & Reset

/************************************************************************************************************************************************
 * Macros to RESET I2Cx Peripheral
 **************************************************************************************************************************************************/

#define I2C1_REG_RESET()				do{(RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1 <<21));}while(0) //Set&Reset
#define I2C2_REG_RESET()				do{(RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1 <<22));}while(0) //Set&Reset
#define I2C3_REG_RESET()				do{(RCC->APB1RSTR |= (1 <<23)); (RCC->APB1RSTR &=~(1 <<23));}while(0) //Set&Reset

/****************************************************************************************************************************************************
 * Macros for RESET USARTx Registers
 */

#define USART1_REG_RESET()		do{ (RCC->APB2RSTR |= ( 1 << 4)); (RCC->APB2RSTR &= ~( 1 << 4));}while(0)//Set&Reset
#define USART2_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 17)); (RCC->APB1RSTR &= ~( 1 << 17));}while(0)//Set&Reset
#define USART3_REG_RESET()		do{ (RCC->APB1RSTR |= ( 1 << 18)); (RCC->APB1RSTR &= ~( 1 << 18));}while(0)//Set&Reset
#define UART4_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 19)); (RCC->APB1RSTR &= ~( 1 << 19));}while(0)//Set&Reset
#define UART5_REG_RESET()			do{ (RCC->APB1RSTR |= ( 1 << 20)); (RCC->APB1RSTR &= ~( 1 << 20));}while(0)//Set&Reset
#define USART6_REG_RESET()		do{ (RCC->APB2RSTR |= ( 1 << 5)); (RCC->APB2RSTR &= ~( 1 << 5));}while(0)//Set&Reset


/************************************************************************************************************************************************
 * Interrupt IRQ configuration number Macros (Position)
 */
#define  IRQ_NO_EXTI0 							6
#define  IRQ_NO_EXTI1							7
#define  IRQ_NO_EXTI2							8
#define  IRQ_NO_EXTI3							9
#define  IRQ_NO_EXTI4							10
#define  IRQ_NO_EXTI9_5						23
#define  IRQ_NO_EXTI15_10					40
#define IRQ_NO_I2C1_EV						31
#define IRQ_NO_I2C1_ER						32
#define IRQ_NO_I2C2_EV						33
#define IRQ_NO_I2C2_ER						34
#define IRQ_NO_I2C3_EV						72
#define IRQ_NO_I2C3_ER						73












//Some Generic Macros

#define ENABLE							1
#define DISABLE							0
#define SET 								ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET					RESET
#define FLAG_SET						SET


// GPIO_BASEADDR_TO_PORTCODE Macro
#define GPIO_BASEADDR_TO_PORTCODE(x) 		((x==GPIOA)? 0 :\
																				(x==GPIOB) ? 1 :\
																				(x==GPIOC) ? 2 :\
																				(x==GPIOD) ? 1 :\
																				(x==GPIOE) ? 1 :\
																				(x==GPIOF) ? 1 :\
																				(x==GPIOG) ? 1 :\
																				(x==GPIOH) ? 1 :0 )

/*************************************************************************************************************************************************
 * 	Bit Position Definition for SPI Peripheral
************************************************************************************************************************************************ */
/***************************************************************************************************************************************************
 * Bit Positions for CR1 register
 */
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL     			1
#define SPI_CR1_MSTR 				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_ BIDIOE 		    14
#define SPI_CR1_BIDIMODE		15

/*
 * Bit Positions for CR2 register
 */
#define SPI_CR2_RXDMAEN 		0
#define SPI_CR2_TXDMAEN 		1
#define SPI_CR2_SSOE		 		2
#define SPI_CR2_FRF			 		4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE				7




/*
 * Bit Positions for SPI status Register
 */
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF				5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/*
 * Bit Positions for I2C CR1 Register
 */
#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS			1
//bit 2 Reserved
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
//bit 14 Reserved
#define I2C_CR1_SWRST			15

/*
 * Bit Positions for I2C_CR2 Register
 */
#define I2C_CR2_FREQ				0
//bit 6 to 7 Revered
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST				12
//bit 13 to 15 Reserved

/*
 * Bit Positions for I2C_SR1 Register
 */
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
//bit 5 Reserved
#define I2C_SR1_RxNE				6
#define I2C_SR1_TxE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR			12
//Bit 13 Reserved
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT		15

/*
 * Bit Positions for I2C_SR2 Register
 */
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
//Bit 3 reserved
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8

/*
 * Bit Field positions for CCR Register
 */
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15

//***********************************************************************************************************************************************


/*-----------------------------------------------------------------------------------------------------------------------------------------------
 *  Bit positions for USART peripheral
 */

/*
 * Bit positions for USART CR1 Register
 */
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

/*
 * Bit Positions for USART_CR2
 */
#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14

/*
 * Bit positions for USART_CR3
 */


#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL		 	3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

/*
 * USART Flag bit positions
 */

#define USART_SR1_PE			0
#define USART_SR1_FE			1
#define USART_SR1_NF			2
#define USART_SR1_ORE			3
#define USART_SR1_IDLE			4
#define USART_SR1_RXNE			5
#define USART_SR1_TC			6
#define USART_SR1_TXE			7
#define USART_SR1_LBD			8
#define USART_SR1_CTS			9











#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_Driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"



#endif /* INC_STM32F446XX_H_ */
