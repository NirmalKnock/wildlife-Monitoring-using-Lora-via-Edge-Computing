/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Dec 6, 2020
 *      Author: Nirmal Kumar
 */

#include <stm32f446xx_gpio_driver.h>
#include <stm32f446xx.h>




/*
 * Peripheral Clock configuration
 */

/************************************************************************************************
 *
  * @fn					- GPIO_PeriClockControl
  *
  * @brief				- Which controls the GPIO clock Enable or Disable for the given GPIO port.
  *
  *	@param[in]			-  GPIO base address
  *	@param[in]			-  Enable or Disable value(MACROS)
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDI){

	if(EnorDI == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}

	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}







/*
 * GPIO Init and Deinit
 */

/************************************************************************************************
 *
  * @fn					- GPIO_Init
  *
  * @brief				- Initialize the GPIO port
  *
  *	@param[in]			- GPIO handle structure pointer
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
     GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint32_t temp =0; 			//Take a temporvary variable

	//1) Configure the mode of GPIO Pin(<=3(analog mode macros is called non-interrupt mode)

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		//Non interrupt Mode

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //multi 2 is because each register takes 2 bits to config.
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);						  //Clearing the bit at specific bitfields.
		pGPIOHandle->pGPIOx->MODER |= temp;

	}

	else
	{

		//greater than 3 is called interrupt Mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FT){
			//1. Configure FTSR register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RT){
			//1. Configure the RTSR Register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);



		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RFT ){
			//1. Configure the Both RTS AND FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}


		//2. Configure the GPIO Port Selection in SYSCFG_Register

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;			// pinnumber/4 (because 4 registers)
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG_PCLK_EN();														// clock enable
		uint8_t portcode = GPIO_BASEADDR_TO_PORTCODE (pGPIOHandle->pGPIOx);		// macro

		SYSCFG->EXTICR[temp1] = portcode << (temp2 *4);

		//3. Enable interrupt Delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}

	//2) Configure the speed
	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing the Bit
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;



	//3) Configure the Pullup/PullDown
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4) Configure the Optype
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp =0;

	//5) Configure Alternate function register
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alternate function
		// Creating the Two variables

		uint8_t temp1,temp2 = 0;

		temp1= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~ (0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2) ) ;

	}


	}

/************************************************************************************************
 *
  * @fn					- GPIO_DeInit
  *
  * @brief				- DeInitialize the GPIO port using the RCC_Reset_control Register
  *
  *	@param[in]			-  Base address of Register peripheral
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)													//DeInitialize the GPIO for we can reset in one shot using RCC_RESET REGISTER(Base address of the Register peripheral)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}




}
/*
 * Data Read and Write
 */

/************************************************************************************************
 *
  * @fn					- GPIO_ReadFromInputPin
  *
  * @brief				- Read the input from the GPIO pin.
  *
  *	@param[in]			- Base address of the GPIO register peripheral
  *	@param[in]			- PinNumber of the GPIO port
  *	@param[in]			-
  *
  *	return				- uint8_t ( either 1 or 0 )
  *
  *	@Note				- none

*/

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)				//ReadFrom input Pin(Base address of the GPIO and Pin Number(pin state is 1 or 0 so ;uint8_t)
{
	uint8_t value;
	value = (uint8_t )((pGPIOx->IDR >> PinNumber) & 00000001);							//IDR register left shifted by pin number and masked to read particular bit field.
	return value;

}



/************************************************************************************************
 *
  * @fn					- GPIO_ReadFromInputPort
  *
  * @brief				- Read the input from the GPIO Port.
  *
  *	@param[in]			- Base address of the GPIO register peripheral
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- uint16_t ( Total of 16 Pins in a port)
  *
  *	@Note				- none

*/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)										//Read from Input Port(Base address of GPIO(total 16 pins so uint_16)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;


}
/************************************************************************************************
 *
  * @fn					- GPIO_WriteToOutputPin
  *
  * @brief				- Write the Output from the GPIO Pin.
  *
  *	@param[in]			- Base address of the GPIO register peripheral
  *	@param[in]			- PinNumber of the GPIO port
  *	@param[in]			- Output Value (1 OR 0)
  *
  *	return				- none
  *
  *	@Note				- none

*/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value) 	//Write to Output Pin(Base address of the GPIO and Pin Number,value(1 or 0)(void because it's write)
{
	if(value == GPIO_PIN_SET)
	{
		//Write 1 as the value to the register at the bit field with the pinNumber
		pGPIOx->ODR |= (1 << PinNumber);
	}

	else
	{
		//Write 0 as the value to the register
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
/************************************************************************************************
 *
  * @fn					- GPIO_WriteToOutputPort
  *
  * @brief				- Write the Output from the GPIO Port.
  *
  *	@param[in]			- Base address of the GPIO register peripheral
  *	@param[in]			-
  *	@param[in]			- Output Value (1 OR 0)
  *
  *	return				- none
  *
  *	@Note				- none

*/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)				//Write to Output Port(Base address of the GPIO and Pin Number(void because it's write,total 16 pins so uint_16)
{
	pGPIOx->ODR = value;

}
/************************************************************************************************
 *
  * @fn					- GPIO_ToggleOutputPin
  *
  * @brief				- Toggling the OUTPUT Pin.
  *
  *	@param[in]			- Base address of the GPIO register peripheral
  *	@param[in]			- PinNumber of the GPIO port.
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void     GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)				//Toggle the output pin(Base address of the GPIO and Pin Number)
{
	pGPIOx->ODR ^= (1 << PinNumber);





}
/*
 * IRQ configuration and ISR Handling
 */

/************************************************************************************************
 *
  * @fn					- GPIO_IRQConfig
  *
  * @brief				- GPIO IRQ configuration function
  *
  *	@param[in]			- IRQ Number (uint8_t(0-255))
  *	@param[in]			- IRQ Priority.
  *	@param[in]			- IRQ Enable or Disable
  *
  *	return				- none
  *
  *	@Note				- none

*/

void GPIO_IRQInterruptConfig(uint8_t IRQnumber, uint8_t EnorDI){				//Setting the priority of the IRQ(IRQ number, Priority, EnorDI)

	if (EnorDI == ENABLE){

		if(IRQnumber <=31)
		{
			//Enable the ISER0   0 to 31
			*NVIC_ISER0 |= (1 << IRQnumber);  // Dereference from Macro

		}

		else if (IRQnumber > 31 && IRQnumber < 64 )
		{
			//Enable the ISER1  32 to 63
			*NVIC_ISER1 |= (1 << (IRQnumber % 32) ); //  Because starts from 32 refer,notes.
		}

		else if (IRQnumber  >=  64 && IRQnumber < 96)
		{
			//Enable the ISER2 65 to 95
			*NVIC_ISER2 |= (1 << (IRQnumber % 64) ); //Because starts with 64 refer notes.
		}


	}else

	{
		if(IRQnumber <=31)
		{
					//Disable the ICER0 0 to 31
			*NVIC_ICER0 |= (1 << IRQnumber);
		}

		else if (IRQnumber > 31 && IRQnumber < 64 )
		{
					//Disable the ICER1 32 to 63
			*NVIC_ICER1 |= (1 << (IRQnumber % 32) );
		}

		else if (IRQnumber <64 && IRQnumber < 96)
		{
			       //Disable the ICER2 65 to 95
			*NVIC_ICER2 |= (1 << (IRQnumber % 64) );

		}

	}

}

/************************************************************************************************
 *
  * @fn					- GPIO_IRQPriority
  *
  * @brief				- Handles GPIO priority
  *
  *	@param[in]			- IRQ  Number
  *	@param[in]			- IRQ Priority from 0 to 15
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- Uses the processor specific registers IRQ priority registers

*/



void GPIO_IRQPriorityConfig(uint8_t IRQnumber,uint32_t IRQPriority)
{

	//1. We want to find IPR register,So we take variable

	uint8_t iprx = IRQnumber / 4;					//Because each sections contains four Register to select which column
	uint8_t iprx_section = IRQnumber % 4;	// then which section.

	uint8_t shift_amount = (8 * iprx_section) + (8 - No_PR_BITS_IMPLEMENTED);
	*( NVIC_PR_BASE_ADDR + iprx   )  |=  (IRQPriority << shift_amount ) ;



}



/************************************************************************************************
 *
  * @fn					- GPIO_IRQHandling
  *
  * @brief				- GPIO IRQHandling function(sub routine)
  *
  *	@param[in]			- GPIO pin Number
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- This takes pin number to find which pin makes the Interrupt.

*/

void GPIO_IRQHandling(uint8_t PinNumber){												//IRQ handling to know ,which pin the IRQ is triggered.

	// Clear the PR register in EXTI corresponding to the pin Number

	if(EXTI->PR & (1 << PinNumber))										// if set at the register ,it indicates the pending ,so we clear that first.
	{

		// Clear the Bit
		EXTI->PR  |= (1 << PinNumber);

	}

}
