/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Dec 6, 2020
 *      Author: Nirmal Kumar
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include <stm32f446xx.h>
#include <stdint.h>





//Creating a config structure
typedef struct {

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				/*|< Possible values from @GPIO PIN MODES >*/
	uint8_t GPIO_PinSpeed;				/*|< Possible values from @GPIO SPEED >*/
	uint8_t GPIO_PinPuPdControl;		/*|< Possible values from @GPIO PUPD >*/
	uint8_t GPIO_PinOPType;				/*|< Possible values from @@GPIO TYPE >*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;





/*
 * This is a Handle Structure for GPIO Pin
 */

typedef struct{

	//Pointer to hold the base address of the GPIO peripheral

	GPIO_RegDef_t 		*pGPIOx; 																// This holds the base address of the peripheral which we use
	GPIO_PinConfig_t 	GPIO_PinConfig;													//This Variable holds the GPIO configurations
}GPIO_Handle_t;


/* @GPIO PIN NUMBERS
 * The possible GPIO Pin Numbers
 */
#define GPIO_PIN_NUMBER_0 			0
#define GPIO_PIN_NUMBER_1 			1
#define GPIO_PIN_NUMBER_2			2
#define GPIO_PIN_NUMBER_3 			3
#define GPIO_PIN_NUMBER_4 			4
#define GPIO_PIN_NUMBER_5 			5
#define GPIO_PIN_NUMBER_6 			6
#define GPIO_PIN_NUMBER_7 			7
#define GPIO_PIN_NUMBER_8 			8
#define GPIO_PIN_NUMBER_9 			9
#define GPIO_PIN_NUMBER_10 		10
#define GPIO_PIN_NUMBER_11 		11
#define GPIO_PIN_NUMBER_12 		12
#define GPIO_PIN_NUMBER_13 		13
#define GPIO_PIN_NUMBER_14 		14
#define GPIO_PIN_NUMBER_15			15











/* @GPIO PIN MODES
 * The possible Modes of the GPIO PIN(Moder Register)
 */
#define GPIO_MODE_IN	 								0
#define GPIO_MODE_OUT								1
#define GPIO_MODE_ALTFN							2
#define GPIO_MODE_ANALOG						3
#define GPIO_IT_FT 										4					//Input falling Edge
#define GPIO_IT_RT										5					//Input Raising Edge
#define GPIO_IT_RFT										6					//Input Raising falling Edge

/* @GPIO TYPE
 * GPIO Pin possible OUTPUT Type (port output type register)
 */
#define GPIO_OP_TYPE_PP								0						// Output Pushpull
#define GPIO_OP_TYPE_OD							1						// Output Opendrain

/* @GPIO SPEED
 * GPIO Pin possible OUTPUT Speed (output speed register)
 */
#define GPIO_SPEED_LOW								0
#define GPIO_SPEED_MEDIUM						1
#define GPIO_SPEED_FAST								2
#define GPIO_SPEED_HIGH							3

/* @GPIO PUPD
 * GPIO Pin possible PULLUP/PULLDown config (pull-up/pull-down register)
 */
#define GPIO_NO_PUPD									0						//No pullup-pullldown
#define GPIO_PIN_PU										1						// PullUp
#define GPIO_PIN_PD										2						// PullDown







/************************************************************************
 * 					API's Supported my this driver
 ***********************************************************************/

/*
 * Peripheral Clock configuration
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDI);										 //Control the peripheral clock,Takes (GPIO baseaddress and Enable or disble)

/*
 * GPIO Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);																			//Initialize the GPIO(base address of Handle structure
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);																					//DeInitialize the GPIO for we can reset in one shot using RCC_RESET REGISTER(Base address of the Register peripheral)

/*
 * Data Read and Write
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);						//ReadFrom input Pin(Base address of the GPIO and Pin Number(pin state is 1 or 0 so ;uint8_t)
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);													//Read from Input Port(Base address of GPIO(total 16 pins so uint_16)
void     GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value);	//Write to Output Pin(Base address of the GPIO and Pin Number,value(1 or 0)(void because it's write)
void     GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);								//Write to Output Port(Base address of the GPIO and Pin Number(void because it's write,total 16 pins so uint_16)
void     GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);							//Toggle the output pin(Base address of the GPIO and Pin Number)

/*
 * IRQ configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQnumber, uint8_t EnorDI);															//Setting the priority of the IRQ(IRQ number EnorDI)
void GPIO_IRQPriorityConfig(uint8_t IRQnumber ,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);																				//IRQ handling to know ,which pin the IRQ is triggered.

















#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
