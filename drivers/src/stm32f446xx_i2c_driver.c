/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: May 18, 2021
 *      Author: Nirmal Kumar
 */
#include "stm32f446xx.h"



/****************************************Function Prototypes****************************************************/
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2C_Handle);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
void I2C_ManagaeAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
void I2C_ClearSBFlag(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandlerTxEInterrupt(I2C_Handle_t *pI2C_Handle);
static void I2C_MasterHandlerRxNEInterrupt(I2C_Handle_t *pI2C_Handle);


/*********************************************************Helper Functions***************************************/
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	//shift register by 1 , to fill r/w bit
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr &= ~(1);							//To clear the first bit (Slave address + r/w bit ) W=0;
	//put into Data buffer
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	//Shift the register by 1, to fill R/W bit
	SlaveAddr = SlaveAddr <<1;
	SlaveAddr |= 1;
	//Put into Data Register
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2C_Handle)						//For non blocking api's refer blocking api , before
{
	uint32_t dummyRead;

	//Check for the device is Master mode or Slave mode
	if ( pI2C_Handle->pI2Cx->SR2 & ( 1<< I2C_SR2_MSL) )
	{
		//The Device in Master Mode***

			//Lets first check the staete
			if ( pI2C_Handle->TxRxState == I2C_BUSY_IN_RX)
			{
					//Then lets check the Length
					if ( pI2C_Handle->RxSize == 1)
					{
							//Then diable the ACK
							I2C_ManagaeAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);

							//Then clear the ADDR flag(Read SR1, followed by SR2)
							dummyRead = pI2C_Handle->pI2Cx->SR1;
							dummyRead= pI2C_Handle->pI2Cx->SR2;
							(void )dummyRead;

					 }
			 }
			else  //if not in Rx , then it excetues this
			{				//Then clear the ADDR flag(Read SR1, followed by SR2)
							dummyRead = pI2C_Handle->pI2Cx->SR1;
							dummyRead =  pI2C_Handle->pI2Cx->SR2;
							(void )dummyRead;
			}


	}
	else
	{
		//The Device in Slave Mode

		//Then clear the ADDR flag(Read SR1, followed by SR2)
									dummyRead = pI2C_Handle->pI2Cx->SR1;
									dummyRead = pI2C_Handle->pI2Cx->SR2;
									(void )dummyRead;

	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1  |= ( 1 << I2C_CR1_STOP );
}

void  I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		//Enable all the interrupt register
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
	}
	else
	{
		pI2Cx->I2C_CR2 &= 	~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= 	~( 1 << I2C_CR2_ITERREN);
		pI2Cx->I2C_CR2 &= 	~( 1 << I2C_CR2_ITEVTEN);
	}
}



void I2C_ManagaeAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_ACK);		//Enable
	}
	if (EnorDi == I2C_ACK_DISABLE)
	{
		pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_ACK); //Disable
	}
}

void I2C_ClearSBFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t temp = pI2Cx->SR1;
	(void)temp;
}

/************************************************************************************************
 *
  * @fn					- I2C_PeriClockControl
  *
  * @brief				- Which controls the I2C clock Enable or Disable.
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or Disable value(MACROS)
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDI)
{
	if (EnorDI == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}

	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/************************************************************************************************
 *
  * @fn					- I2C_init()
  *
  * @brief				- Which initiazise the I2C peripheral
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or Disable value(MACROS)
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void I2C_Init(I2C_Handle_t  *pI2C_Handle_t)
{
	//.Creating the temp variable and trise variable

	uint32_t tempreg = 0 ;

	//Enable the I2C peripheral Clock
	I2C_PeriClockControl(pI2C_Handle_t->pI2Cx, ENABLE);


	//1. ACK control Bit
	//tempreg |= (pI2C_Handle_t->I2C_Config.I2C_ACKControl   << I2C_CR1_ACK) ;
	 pI2C_Handle_t->pI2Cx->I2C_CR1 |= ( 1 << 10);

	//2. Configure the FREQ field

	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U ;   //Because we want only the number. eg 16
	pI2C_Handle_t->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	//3. Configure the device own  address in OWN register

	tempreg= 0 ;
	tempreg |= (pI2C_Handle_t->I2C_Config.I2C_DeviceAddress  << 1 );  // Because 1st bit is dont care in 7bit addressing Mode
	//Read reference manual 14th bit kept at 1 by software (Reserved)
	tempreg |= (1 << 14);
	pI2C_Handle_t->pI2Cx->OAR1 = tempreg;
	//Read reference manual 14th bit kept at1 by software

	//4. CCR calculation

	uint16_t ccr_value = 0;
	tempreg = 0;

	//First to find the Speed mode

	if (pI2C_Handle_t->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Then it is standard Mode
		//By default the standard Mode is configured in CCR Register in 15th Bit
		//calculation for CCR in SM mode

		ccr_value = (RCC_GetPCLK1Value() / (2* pI2C_Handle_t->I2C_Config.I2C_SCLSpeed) );  // CCR = pclock / (2* SCK by user)
		tempreg |= (ccr_value & 0xFFF); //masking only ccr bit fields

	}
	else
	{
		//the mode is Fast Mode
		//Set fast mode in Bit 15th

		tempreg |= (1 << 15);
		//then set the duty cycle

		tempreg |= (pI2C_Handle_t->I2C_Config.I2C_FMDuty_Cycle <<  14 );
		//Based on duty cycle use formular

		if (pI2C_Handle_t->I2C_Config.I2C_FMDuty_Cycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2C_Handle_t->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2C_Handle_t->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2C_Handle_t->pI2Cx->CCR |= tempreg;


		//5.  TRISE configuration
		//First check the Mode Sm or Fm

	if (pI2C_Handle_t->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM)
	{
		//Mode is Standard Mode
		tempreg = 0;
		tempreg = (RCC_GetPCLK1Value()  / 1000000U ) + 1 	;//Formula : (Pclk1 * Trise)+1 ,  we can simply say , trise = 1us ,so we divide by 1MHZ


	}
	else
	{
		//Mode is Fast Mode
		tempreg =0;
		tempreg = ( (RCC_GetPCLK1Value() * 300 ) / 1000000000U ) +1;					//Fast Mode trise = 300ns
	}

	pI2C_Handle_t->pI2Cx->TRISE |= (tempreg & 0x3F);


}


/***********************************************************************************************************************************
 *
  * @fn					- I2C_DeInit
  *
  * @brief				- Which Reset the I2C peripheral
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void I2C_DeInit(I2C_RegDef_t   *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*********************************************************************************************************************************************
 *
  * @fn					- I2C Master Send Data
  *
  * @brief				- Which Transmits the data from I2Cx peripheral
  *
  *	@param[in]			-  Handle Structure of I2Cx
  *	@param[in]			-  Buffer address pointer
  *	@param[in]			-  Length information
  *	@param[in]			-  Slave address information
  *	@param[in]			- Repeated Start
  *
  *	return				- none
  *
  *	@Note				- none

*/

void I2C_MasterSendData (I2C_Handle_t *pI2C_Handle_t , uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr )
{
	//1.	Generate the Start Conditioin(creating helper function)
	I2C_GenerateStartCondition(pI2C_Handle_t->pI2Cx);

	//2. 	Confirm the start Generation is completed by checking the SB flag in the SR1 Register
	//		note : Utill the SB is cleared the SCL will be stretched (Pulled to Low)
	//		wait untill the SB is cleared
	while( !  I2C_Get_FlagStatus(pI2C_Handle_t->pI2Cx, I2C_FLAG_SR1_SB) );


	//3.	Send the Address of the slave with r/w bit , Set to W(0)  (Total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2C_Handle_t->pI2Cx, SlaveAddr);

	//4. 	Confirm the address phase is completed by Checking in ADDR flag in SR1 register
	while( !  I2C_Get_FlagStatus(pI2C_Handle_t->pI2Cx, I2C_FLAG_SR1_ADDR) );

	//5.	 Clear the ADDR flag according to its software Sequence
	//		 ADDR FLAG can be cleared by reading SR1 and SR2 , check RM in SR1 register
	I2C_ClearADDRFlag(pI2C_Handle_t);

	//6.	Send the data untill Len becomes =0
	//		before sending the data , first check the TXE flag ,whether the data buffer is empty or not

	while (Len >0)
	{
		while ( ! I2C_Get_FlagStatus(pI2C_Handle_t->pI2Cx, I2C_FLAG_SR1_TXE));		//Wait Till TXE is SET
		pI2C_Handle_t->pI2Cx->DR = *pTxbuffer;
		pTxbuffer ++;
		Len--;
	}

	//7.	When Len becomes 0, wait for TXE=1 and BTF =1 before Generating the STOP condition
	//		Note :  TXE=1 and BTF = 1, means the SR and DR registers are empty and next transmission should begin
	//		when  BTF = 1  SCL will be stretched (Pulled to LOW)

	while ( ! 	(  I2C_Get_FlagStatus(pI2C_Handle_t->pI2Cx, I2C_FLAG_SR1_TXE) ) );

	while ( !   ( I2C_Get_FlagStatus(pI2C_Handle_t->pI2Cx, I2C_FLAG_SR1_BTF) ) );

	//8. 	Generate the STOP Condition and master need not to wait for the completion of STOP Condition
	//		Note: Generating STOP , will automatically clears the BTF

	if(Sr == I2C_DISABLE_SR)
	{
	I2C_GenerateStopCondition(pI2C_Handle_t->pI2Cx);

	}





}


/**********************************************************************************************************************************
 *
  * @fn					- I2C_MasterReceiverData
  *
  * @brief				- Which Receives the data from the Slave with 1 byte option and multiple byte option
  *
  *	@param[in]			- I2C_Handle_t structure
  *	@param[in]			-  RxBuffer address
  *	@param[in]			- Length Information
  *	@param[in]			- Slave address
  *	@param[in]			- Repeated Start
  *
  *	return				- none
  *
  *	@Note				- none

*/

void I2C_MasterReceiverData( I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t len, uint8_t Slaveaddress,uint8_t Sr)
{
	//1.	Generate Start Condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	//2.	Confirm the start Condition by checking the SB flag  in SR1 register
	//Note: 	Ultill the SB bit is cleared the SCL line will be Stretched Low(pulled to Low)
	while ( ! I2C_Get_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SR1_SB));

	//3.	Send the Address of the slave in 7bit mode with Read information bit R=1. (Total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2C_Handle->pI2Cx, Slaveaddress);

	//4. Wait Untill the Address phase is completed by checking the ADDR Flag in SR1 register

	while ( ! I2C_Get_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SR1_ADDR));

	/// Procedure to Read Only one Byte

	if (len ==1)
	{
		//Disable the ACK
		I2C_ManagaeAcking(pI2C_Handle->pI2Cx,I2C_ACK_DISABLE);

		//Clear the ADDR flag ,by reading SR1 followed by the SR2, check RM
		I2C_ClearADDRFlag(pI2C_Handle);


		//Wait untill the RxNE becomes 1
		while ( ! I2C_Get_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SR1_RXNE));

		//Generate Stop Condition
		if (Sr == I2C_DISABLE_SR)
		{
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
		}

		//Read Data into the buffer
		*pRxBuffer = pI2C_Handle->pI2Cx->DR;  			//for only 1 byte , no need for buffer++(increment)


	}

	//	Procedure if the length is greater than one  (len >1)
	if( len >1)
	{
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2C_Handle);

		for ( uint32_t i= len; i >0  ; i --)
		{

			//Wait untill the RxNE becomes 1
			while (! I2C_Get_FlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SR1_RXNE));

			if ( i == 2)
			{
				//	Clear the Ack bit
				I2C_ManagaeAcking(pI2C_Handle->pI2Cx,I2C_ACK_DISABLE);

				//	Generate the Stop Bit
				if (Sr == I2C_DISABLE_SR)
				{
				I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
				}

			}
			//Read the data from Data Register into the buffer
			*pRxBuffer = pI2C_Handle->pI2Cx->DR;
			pRxBuffer++;


		}

	}

	//Re-Enable the ACK
	if (pI2C_Handle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) 		//if ack is enabled in structure ,then only enables the Ack in register
	{
	I2C_ManagaeAcking(pI2C_Handle->pI2Cx,I2C_ACK_ENABLE);
	}

}

/**********************************************************************************************************************************
 *
  * @fn					- I2C_SlaveSendData
  *
  * @brief				- Which Sends the data from the Slave to Master
  *
  *	@param[in]			- I2C_RegDef_t structure
  *	@param[in]			-  data
  *
  *	return					-
  *
  *	@Note				- none

*/

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data)																											//We send data in byte -byte, so on need for other parameters like mastersend data
{
	pI2Cx->DR = data;

}

/**********************************************************************************************************************************
 *
  * @fn					- I2C_SlaveReceiverData
  *
  * @brief				- Which Receives data from Master
  *
  *	@param[in]			- I2C_RegDef_t structure
  *	@param[in]			-  data
  *
  *	return					-  uint8_t
  *
  *	@Note				- none

*/


uint8_t I2C_SlaveReceiverData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR ;
}



/**********************************************************************************************************************************
 *
  * @fn					- I2C_MasterSendDataIT
  *
  * @brief				- Which Sends the data from the Master on the Interrupt
  *
  *	@param[in]			- I2C_Handle_t structure
  *	@param[in]			-  RxBuffer address
  *	@param[in]			- Length Information
  *	@param[in]			- Slave address
  *	@param[in]			- Repeated Start
  *
  *	return					- Which return the State
  *
  *	@Note				- none

*/

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2C_Handle , uint8_t *pTxBuffer , uint32_t Len, uint8_t Slaveaddress,uint8_t Sr)
{
	uint8_t busystate;
	busystate = pI2C_Handle->TxRxState;

	//checking if I2C busy or Not
	if ( ( busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) )
	{
		pI2C_Handle->pTxBuffer = pTxBuffer;
		pI2C_Handle->TxLen = Len;
		pI2C_Handle->TxRxState = I2C_BUSY_IN_TX;
		pI2C_Handle->DevAddr = Slaveaddress;
		pI2C_Handle->Sr = Sr;

		//1.	Implement the code to generate start condition
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		//2.	Implement the code to enable ITBUFEN in Control Register 2
		pI2C_Handle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//	3. Implement the code to enable ITEVFEN in Control Register 2
		pI2C_Handle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//4. Implement the code to enable ITERREN in Control Register 2
		pI2C_Handle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}
	//if above is not happen then continues this
	return busystate;

}


/**********************************************************************************************************************************
 *
  * @fn					- I2C_ReceiveDataIT
  *
  * @brief				- Which Receives the data during Interrupt
  *
  *	@param[in]			-  Address of I2C handle structure
  *	@param[in]			-  Rx buffer addres
  *	@param[in]			- Length
  *	@param[in]			- Slave address
  *	@param[in]			- Repeated start Enable or disable
  *	return				- none
  *
  *	@Note				- none

*/

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Handle ,  uint8_t  *pRxBuffer , uint32_t Len, uint8_t Slaveaddress,uint8_t Sr )
{
	//	Lets first check the I2C is busy or not

	uint8_t busystate = pI2C_Handle->TxRxState;

	if ( (busystate != I2C_BUSY_IN_TX ) && ( busystate != I2C_BUSY_IN_RX))
	{
		pI2C_Handle->pRxBuffer = pRxBuffer;
		pI2C_Handle->RxLen= Len;
		pI2C_Handle->TxRxState= I2C_BUSY_IN_RX;
		pI2C_Handle->RxSize= Len;
		pI2C_Handle->DevAddr = Slaveaddress;
		pI2C_Handle->Sr= Sr;

		//Implement to code to generate start condition
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		//1.	Implementing the ITBUFEN Register in CR2 register
		pI2C_Handle->pI2Cx->I2C_CR2		 |= ( 1 << I2C_CR2_ITBUFEN);

		//2. Implementing the ITEVEN Register in CR2 register
		pI2C_Handle->pI2Cx->I2C_CR2 	|= ( 1 << I2C_CR2_ITEVTEN);

		//3. Implementing the ITERREN Register in CR2 register
		pI2C_Handle->pI2Cx->I2C_CR2 	|= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


/**********************************************************************************************************************************
 *
  * @fn					- I2C peripheral Control
  *
  * @brief				- Which controls the I2C peripheral
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or disable
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi)
{

	if (EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/**********************************************************************************************************************************
 *
  * @fn					- I2C_IRQInterruptConfig
  *
  * @brief				- Which configure the IRQ in NVIC register (Enabling)
  *
  *	@param[in]			-  IRQ NUMBER
  *	@param[in]			-  Enable or Disble
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
			}
		}
}



/**********************************************************************************************************************************
 *
  * @fn					- I2C_IRQPriorityConfig
  *
  * @brief				- Which Configures the Priority in Interrput priority Register
  *
  *	@param[in]			-  IRQ Number
  *	@param[in]			-  IRQ priority
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void I2C_IRQPriorityConfig(uint8_t IRQnumber ,uint8_t IRQPriority)
{
	uint8_t ipr_section, ipr_bitfiled,shift_amount;

	ipr_section = IRQnumber / 4 ;
	ipr_bitfiled = IRQnumber % 4;

	shift_amount = ( 8 * ipr_bitfiled ) + ( 8 - No_PR_BITS_IMPLEMENTED);				//because each register contains 8 bit field

	*(NVIC_PR_BASE_ADDR + ipr_section) |= ( IRQPriority << shift_amount);

}


/**********************************************************************************************************************************
 *
  * @fn					- I2C Get Flag Status
  *
  * @brief				- Which gives the flag status in SR register
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  flag name
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

uint8_t I2C_Get_FlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flag_Name)
{
	if (pI2Cx->SR1 & Flag_Name)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle)
{
	//To close the communication disable the interrupts

	// Disable the ITBUFEN interrupt
	pI2C_Handle->pI2Cx->I2C_CR2 &= ~ ( 1<< I2C_CR2_ITBUFEN);				//Which prevents further interrupt from TXE and RXNE

	//	Disable the ITEVEN interrupt
	pI2C_Handle->pI2Cx->I2C_CR2 &= ~ ( 1<< I2C_CR2_ITEVTEN);				//Which prevents further interrupt from Events

	//after that clear all the structure
	pI2C_Handle->TxRxState = I2C_READY;
	pI2C_Handle->pTxBuffer = NULL;
	pI2C_Handle->RxLen = 0;
	pI2C_Handle->RxSize = 0;



	//after that enble the Acking
	if ( pI2C_Handle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManagaeAcking(pI2C_Handle->pI2Cx, I2C_ACK_ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t  *pI2C_Handle)
{

	//To close the communication disable the interrupts

		// Disable the ITBUFEN interrupt
		pI2C_Handle->pI2Cx->I2C_CR2 &= ~ ( 1<< I2C_CR2_ITBUFEN);				//Which prevents further interrupt from TXE and RXNE

		//	Disable the ITEVEN interrupt
		pI2C_Handle->pI2Cx->I2C_CR2 &= ~ ( 1<< I2C_CR2_ITEVTEN);				//Which prevents further interrupt from Events

		pI2C_Handle->TxRxState = I2C_READY;
		pI2C_Handle->pTxBuffer = NULL;
		pI2C_Handle->TxLen =0;

}




static void I2C_MasterHandlerTxEInterrupt(I2C_Handle_t *pI2C_Handle)
{

	if(pI2C_Handle->TxLen > 0)
		{
			//1. load the data in to DR
		pI2C_Handle->pI2Cx->DR =  *(pI2C_Handle->pTxBuffer);

			//2. decrement the TxLen
		pI2C_Handle->TxLen--;

			//3. Increment the buffer address
		pI2C_Handle->pTxBuffer++;

		}
}

static void I2C_MasterHandlerRxNEInterrupt(I2C_Handle_t *pI2C_Handle)
{
				//threre is a two option that is LEN=1 and LEN >1

				//if Len =1
				if(pI2C_Handle->RxSize == 1 )
				{
					*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->DR;
					pI2C_Handle->RxLen--;
				}


				if(pI2C_Handle->RxSize >1)
				{
							//if the len = 2 then disable ack
							if( pI2C_Handle->RxLen == 2 )
							{
									//Disable the Acking
									I2C_ManagaeAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);
							}

										//Reading data
										*(pI2C_Handle->pRxBuffer)= pI2C_Handle->pI2Cx->DR;
										pI2C_Handle->pRxBuffer++;
										pI2C_Handle->RxLen--;

				}

			//if Len =0 , then close i2c and notify the application
				if ( pI2C_Handle->RxLen == 0)
				{

						if (pI2C_Handle->Sr == I2C_DISABLE_SR)
						{
										//1. Generate stop condition
										I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
						}

										//2. Close the I2Cx
										I2C_CloseReceiveData(pI2C_Handle);

										//3. Notify the application
										I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_RX_CMPLT);


				}
}


/**********************************************************************************************************************************
 *
  * @fn					-I2C_EV_IRQHandling
  *
  * @brief				- Which interrupts the Event IRQ handling
  *
  *	@param[in]			-  Address of Handle Structure
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- Interrupt handling for different I2C events (refer SR1)

*/

void I2C_EV_IRQHandling(I2C_Handle_t *pI2C_Handle)
{
		////Interrupt handling for both master and slave mode of a device (To find which event is producing the

		//Lets create a variables
			uint32_t temp1, temp2, temp3;

		//first check the ITEVFEN is enables or not
			temp1 = pI2C_Handle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITEVTEN);			//ITEVENT check if it is enable

			temp2 = pI2C_Handle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITBUFEN);			//ITBUFEN , check if it is enable

			temp3 = pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);					//check start bit is enabled or not

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
		//1. Handle For interrupt generated by SB event
		//	Note : SB flag is only applicable in Master mode
			if ( temp1 && temp3 )
			{
				//Then this interrupt  is generated by SB event, because SB is generated by master
				//Note : This block will not execute in the slave mode because for Slave SB is always Zero(0)
				//In this block further executes the Address phase

				//based on the application state , we decide write or Read
				if ( pI2C_Handle->TxRxState == I2C_BUSY_IN_TX)									//if busy in Tx, then write, else read
				{
					I2C_ExecuteAddressPhaseWrite(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr);
				}
				else	if (pI2C_Handle->TxRxState == I2C_BUSY_IN_RX)
				{
					I2C_ExecuteAddressPhaseRead(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr);
				}
			}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
		temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
		//2. Handle For interrupt generated by ADDR event
		//Note : When master mode : Address is sent
		//		 When Slave mode   : Address matched with own address
			if ( temp1 &&  temp3 )
			{
				//Then this interrupt is generated by ADDR event
				//this causes the clock stretching , so clear ADDR flag
				I2C_ClearADDRFlag(pI2C_Handle);
			}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/

		temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
		//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
			if ( temp1 && temp3 )
			{
					//Then the interrupt is generated by BTF event
					//first check the application state
					if (pI2C_Handle->TxRxState == I2C_BUSY_IN_TX)
					{
									//make sure that TXE is also Set
										if ( pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_TxE))
										{
																		//Then, it shows BTF and TXE=1, before closing make sure len =0
																		if (pI2C_Handle->TxLen ==0)
																		{
																						if (pI2C_Handle->Sr == I2C_DISABLE_SR)
																						{
																							//1. To close the communication, Generate stop condition
																								I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
																						}


																					//2.	Reset all the member elements in handle structure
																					I2C_CloseSendData(pI2C_Handle );

																					//3.	Notify about the transmission is complete
																					I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_TX_CMPLT);

																		}
										}

						}else if ( pI2C_Handle->TxRxState == I2C_BUSY_IN_RX)
								{
									;
								}
}


/*-----------------------------------------------------------------------------------------------------------------------------------------*/
			temp3 = pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
		//4. Handle For interrupt generated by STOPF event
		// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
			if ( temp1 && temp3 )
			{
				// This interrupt is generated by the STOPF event, and makes stop flag as 1 (read RM).
				//So to clear that , by writing in Cr1 register

				pI2C_Handle->pI2Cx->I2C_CR1 |= 0x0000;			//to clear stopf flag, just do bit wise with 0 in Cr1 register

				//Notify the application that stop is generated by Master( detected)
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_STOP);
			}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
		//It is generated by enableing ITBUFFEN and ITEVNT register
		//5. Handle For interrupt generated by TXE event . if ITBUFEN =1 , when TXE becomes 1
			temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);						//check TXE =1
		//temp1 = ITEVNT, temp2= ITBUFFEN, temp3 = TXE

			if ( temp1 && temp2 && temp3 )
			{
				//Before that check the device in master mode , by MSL bit in SR2 register
				if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
					{
							//This is Genereted by TXE
							//We have to write the data in DR register (check Rm),first check state of the I2C
							if ( pI2C_Handle->TxRxState == I2C_BUSY_IN_TX)
							{
									I2C_MasterHandlerTxEInterrupt(pI2C_Handle);
							}

					}
				else  	//The device in slave mode
				{
							//Make sure that the slave is really in transmitter mode				//0 means receive Mode, 1 transmit mode
							if (pI2C_Handle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
							{
									I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_DATA_REQ);
							}
				}

			}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
			temp3 = pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
		//6. Handle For interrupt generated by RXNE event if ITBUFFEN = 1 , when

			if ( temp1 && temp2 && temp3)
			{
					//Check the device mode is master
					if ( pI2C_Handle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
					{
									//This interrupt is generated by RxNE
									//check the state
									if ( pI2C_Handle->TxRxState == I2C_BUSY_IN_RX)
									{
										I2C_MasterHandlerRxNEInterrupt(pI2C_Handle);
									}
					}
					else
					{
								//The mode is slave Mode
								//To check the slave is in Receiver Mode
								if (! (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))				//For receive mode : is 0 in TRA sr2 register
								{
									I2C_ApplicationEventCallback(pI2C_Handle, I2C_EV_DATA_RCV);
								}
					}
			}
}





/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2C_Handle)
{
	uint32_t temp1 ,temp2;

			//Know the status of ITERREN register is set or not , in CR2 register
			temp2 = pI2C_Handle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITERREN);

/**************************************check for Bus error**************************************************************** */

			temp1 = pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_BERR);

			if ( temp1 && temp2 )
			{
				//This is bus error

				//so , to clear the bus ready, by writing 0 to it
				pI2C_Handle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);				//writing to BEER register

				//Implement the code to nofity the application about the error
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_BERR);
			}

/***************************************check for arbitration lost error******************************************************/
			temp1 = (pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_ARLO));

			if (temp1 && temp2 )
			{
				//This is arbiration loss error by Master

				//To clear that , by writing 0 to it
				pI2C_Handle->pI2Cx->SR1 &= ~ (1 << I2C_SR1_ARLO);

				//To implement the code to notify application about the error
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_ARLO);
			}

/*****************************************Check for ACK failure  error******************************************************/

			temp1 = (pI2C_Handle->pI2Cx->SR1 & (1 << I2C_SR1_AF));

			if (temp1 && temp2 )
			{
				//This is ACK (AF) Error

				//To clear the ACK , writing 0 to it
				pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

				//To implement the code to notify application about the error
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_AF);

			}

/*******************************************Check for Overrun/underrun error*****************************************************/

			temp1= pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_OVR);

			if (temp1 && temp2 )
			{
				//This is OVR error

				//To clear that , writing 0 to it
				pI2C_Handle->pI2Cx->SR1 &= ~ ( 1 << I2C_SR1_OVR);

				//To implement the code to notify the application about the error
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_OVR);
			}

/****************************************Check for Time out error***************************************************************/
			temp1 = (pI2C_Handle->pI2Cx->SR1 & ( 1 << I2C_SR1_TIMEOUT));

			if ( temp1 && temp2 )
			{
				//This is Time Out Error

				//To clear this , writing 0 to it
				pI2C_Handle->pI2Cx->SR1 &= ~ (1 << I2C_SR1_TIMEOUT);

				//To implement the code to notify the application about the error
				I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_TIMEOUT);
			}

}





