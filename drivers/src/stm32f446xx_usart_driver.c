/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Jun 8, 2021
 *      Author: Nirmal Kumar
 */


#include "stm32f446xx.h"
#include "stm32f446xx_usart_driver.h"
#include <string.h>





/**********************************************************************************************************************************
 *
  * @fn					- USART_PeriClockControl
  *
  * @brief				- Which controls the Clock of USART peripheral
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or disable
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void USART_PeriClockControl(USART_RegDef_t *pUSARTx , uint8_t EnorDi)
{
			if (EnorDi == ENABLE)
			{
					if (pUSARTx == USART1)
					{
						USART1_PCLK_EN();
					}
					else if (pUSARTx == USART2)
					{
						USART2_PCLK_EN();
					}
					else if (pUSARTx == USART3)
					{
						USART3_PCLK_EN();
					}
					else if (pUSARTx == UART4)
					{
						UART4_PCLK_EN();
					}
					else if (pUSARTx == UART5)
					{
						UART5_PCLK_EN();
					}
					else if ( pUSARTx == USART6)
					{
						USART6_PCLK_EN();
					}

			}else
			{
					if (pUSARTx == USART1)
						{
							USART1_PCLK_DI()	;
						}
						else if (pUSARTx == USART2)
						{
							USART2_PCLK_DI();
						}
						else if (pUSARTx == USART3)
						{
							USART3_PCLK_DI();
						}
						else if (pUSARTx == UART4)
						{
							 UART4_PCLK_DI();
						}
						else if (pUSARTx == UART5)
						{
							 UART4_PCLK_DI();
						}
						else if ( pUSARTx == USART6)
						{
							USART6_PCLK_DI();
						}
	}
}

/**********************************************************************************************************************************
 *
  * @fn					- USART_PeripheralControl
  *
  * @brief				- Which Enables or disables the USART Peripheral
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or disable
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void USART_PeripheralControl (USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);
	}
	else if (EnorDi == DISABLE)
	{
		pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
	}

}


/**********************************************************************************************************************************
 *
  * @fn					- USART_GetFlagStatus
  *
  * @brief				- Which gets the flag status and return it
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- uint8_t
  *
  *	@Note				- none

*/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint32_t Status_FlagName)
{
		if (pUSARTx->USART_SR & Status_FlagName)
		{
			return FLAG_SET;
		}
		return FLAG_RESET;
}

/*************************************************************************************************
 * @fn						- USART_Clear flag status
 *
 * @brief					- 	Which clears the flag
 *
 *	@param[in]			-  Address of RegDef_t Structure
 *	@param[in]			-
 *	@param[in]			-
 *
 *	return					- uint8_t
 *
 *	@Note					- none

*/
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx,uint16_t FlagName)
{
	pUSARTx->USART_SR &= ~(FlagName);
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Which initialize the USART Peripheral
 *
 * @param[in]         - USART_Handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -None
 *
 * @Note              -

 */


void USART_Init(USART_Handle_t *pUSART_Handle)
{
		//Creating the temporary variable
		uint32_t temp =0;

	/******************************** Configuration of CR1******************************************/
			//Implement the code to enable the Clock for given USART peripheral
			USART_PeriClockControl(pUSART_Handle->pUASRTx, ENABLE);

			//1.	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
			if (pUSART_Handle->pUSART_Config.USART_Mode == USART_MODE_ONLY_TX)
			{
				//Implement the code to enable the Transmitter bit field
				temp |= ( 1 << USART_CR1_TE);
			}
			else if ( pUSART_Handle->pUSART_Config.USART_Mode == USART_MODE_ONLY_RX)
			{

				//Implement the code to enable the Receiver bit field
				temp |= ( 1 << USART_CR1_RE);
			}
			else if (pUSART_Handle->pUSART_Config.USART_Mode == USART_MODE_TXRX)
			{
				//Implement the code to enable the Transmitter and Receiver bit field
				temp |= ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE);
			}

		//2.	Implement the code to configure the Word length configuration item
			temp |= (pUSART_Handle->pUSART_Config.USART_WordLength << USART_CR1_M);

		//3. Configuration of parity control bit fields
			if (pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
			{
				temp |= (1 << USART_CR1_PCE);			//enable the parity register
				//By default parity is EVEN parity
			}
			else if (pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
			{
				temp |= (1 << USART_CR1_PCE);			//enable the parity register
				temp |= ( 1 << USART_CR1_PS);				//Enable ODD parity
			}
			else if (pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				temp &= ~ ( 1 << USART_CR1_PCE);			//disabling the parity register
			}

//Finaly program that CR1 register
			pUSART_Handle->pUASRTx->USART_CR1 = temp;


/******************************** Configuration of CR2******************************************/
			temp =0;

			//Implement the code to configure the number of stop bits inserted during USART frame transmission
			temp |= ( pUSART_Handle->pUSART_Config.USART_NoOfStopBits <<  USART_CR2_STOP);

//Configuring the temp to CR2
			pUSART_Handle->pUASRTx->USART_CR2 = temp;


/******************************** Configuration of CR3******************************************/

			temp = 0;

			//Configuration of USART hardware flow control
			if (pUSART_Handle->pUSART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
			{
				temp |= (1 << USART_CR3_CTSE);				//CTS enable
			}
			else if (pUSART_Handle->pUSART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
			{
				temp |= ( 1 << USART_CR3_RTSE);				//RTS enable
			}
			else if (pUSART_Handle->pUSART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
			{
				temp |= ( 1<< USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE);		//Enable both CTS and RTSE
			}

//Configuring the CR3 in USART register
			pUSART_Handle->pUASRTx->USART_CR3 = temp;



/******************************** Configuration of BRR(Baudrate register)******************************************/

				//Implement the code to configure the baud rate

			USART_SetBaudRate(pUSART_Handle->pUASRTx, pUSART_Handle->pUSART_Config.USART_Baud);











}


/**********************************************************************************************************************************
 *
  * @fn					- USART_DeInit
  *
  * @brief				- Which De inialize the peripheral
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or disable
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if ( pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if ( pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
	else if (pUSARTx ==USART6)
	{
		USART6_REG_RESET();
	}
}



/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -	Which send ths data in 9bit format or 8 bit format
 *
 * @param[in]         -	USART_register baseaddress
 * @param[in]         -	Txbuffer address
 * @param[in]         -	Len information
 *
 * @return            - None
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSART_Handle,uint8_t *pTxBuffer,uint32_t Len)
{
	uint16_t *pData;

	//Loop over untill all the data is transfered
			for (uint32_t i =0; i <Len; i++)
			{

				//Implement the code to wait ultill the TXE is set to 1 (data register is Empty)
				while ( ! USART_GetFlagStatus(pUSART_Handle->pUASRTx, USART_FLAG_TXE));

				//Check the Format is 9bit or 8 bit format
				if (pUSART_Handle->pUSART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if it 9 bit Load 2 bytes of data and masking bits other than 9 bits
					pData = (uint16_t*)pTxBuffer;
					pUSART_Handle->pUASRTx->USART_DR = (*pData & (uint16_t)0x01FF); 	//masking other bits except 9 bits and load

							//check for USART_ParityControl
								if (pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_DISABLE)
								{
									//if disabled , all the 9 bits are transfered by the peripheral
									//Code to increse the buffer address twice.
									pTxBuffer++;
									pTxBuffer++;
								}
								else
								{
									//Parity bit is automatically set by a parity register , no need to configure here and 8 bits are transfered
									// increment buffer address once
									pTxBuffer++;
								}

				}
				else
				{
					//This is 8bit format, so 8 bit will transfer
					pUSART_Handle->pUASRTx->USART_DR = (*pTxBuffer & (uint8_t) 0xFF);
					//Code to increase buffer address
					pTxBuffer++;
				}
			}
			//Check the transmission is complete
			while ( ! USART_GetFlagStatus(pUSART_Handle->pUASRTx, USART_FLAG_TC));

}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -	 Which sends the data
 *
 * @param[in]         - USART register baseaddress
 * @param[in]         - Rxbuffer Base address
 * @param[in]         - Len information
 *
 * @return            -	None
 *
 * @Note              -

 */
void USART_ReceiveData(USART_Handle_t *pUSART_Handle,uint8_t *pRxBuffer,uint32_t Len)
{
	//lets make a loop to receive all data untill LEN =0
			for (uint32_t i =0; i <Len; i++)
			{

				//1. First check the RXNE is set or not, RXNE=1 , data register is full, waiting for Read
				while (! USART_GetFlagStatus(pUSART_Handle->pUASRTx, USART_SR1_RXNE));

				//2. Check the receive mode is 9bit format or 8bit fomrat
				if (pUSART_Handle->pUSART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//9 bit format
					//So , 8 bit is user data and 1 bit is for parity

								// Lets check the parity
								if ( pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_DISABLE)
								{
									//parity disables for all 9 bits are user data
									//read only the specific 9 bits form 16bit data
									*((uint16_t*)pRxBuffer) = ( pUSART_Handle->pUASRTx->USART_DR  & ((uint16_t) 0x01FF) );

									//Lets increase the buffer address
									pRxBuffer++;
									pRxBuffer++;

								}
								else
								{
									//Parity is selected , then parity is managed by parity register
									//so just  8 bits of user data and 1 bit of parity bit, Read only  8bits
									*pRxBuffer = (pUSART_Handle->pUASRTx->USART_DR & ( (uint8_t )0xFF));
									pRxBuffer++;

								}

			  	}	else
					{
								//Word length is 8 .

								//First check the parity is enabled or not
								if (pUSART_Handle->pUSART_Config.USART_ParityControl == DISABLE)
								{
									//Parity is disabled
									//all the 8 bits are usedata, so read all 8 bits
									*pRxBuffer = pUSART_Handle->pUASRTx->USART_DR;
								}
								else
								{
									//parity is set, so only 7 bits are user data, 1 bit is parity managed by parity register
									//So , only read 7 bits of user data masking with 0x7F
									*pRxBuffer = (pUSART_Handle->pUASRTx->USART_DR & ((uint8_t)0x7F));
									pRxBuffer++;

								}
					}
			}
}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             -	Which sends the data , in interrupt Mode
 *
 * @param[in]         -	Usart handle strcuture
 * @param[in]         -	Txbuffer address
 * @param[in]         -	Len information
 *
 * @return            -	None
 *
 * @Note              - Enabling the interrupt after the configuration

 */

uint8_t USART_SendDataIT(USART_Handle_t *pUSART_Handle,uint8_t *TxBuffer,uint32_t Len)
{

	uint8_t txstate = pUSART_Handle->TxBusyState;

				if (txstate != USART_BUSY_IN_TX)
				{
					pUSART_Handle->TxLen = Len;
					pUSART_Handle->TxBusyState = USART_BUSY_IN_TX;
					pUSART_Handle->pTxBuffer = TxBuffer;


					//Code implentation for enabling the interrupt for TXE
					pUSART_Handle->pUASRTx->USART_CR1 |= ( 1 << USART_CR1_TXEIE);


					//Code implentation for Enabling the interrupt for TC
					pUSART_Handle->pUASRTx->USART_CR1 |= ( 1 << USART_CR1_TCIE);

				}
	return txstate;

}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -	Which Receives  the data , in interrupt Mode
 *
 * @param[in]         -	Usart handle strcuture
 * @param[in]         -	Txbuffer address
 * @param[in]         -	Len information
 *
 * @return            -	None
 *
 * @Note              - Enabling the interrupt after the configuration

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSART_Handle,uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t rxstate = pUSART_Handle->RxBusyState;

				if (rxstate != USART_BUSY_IN_RX)
				{
					pUSART_Handle->RxLen = Len;
					pUSART_Handle->pRxBuffer = pRxBuffer;
					pUSART_Handle->RxBusyState = USART_BUSY_IN_RX;

					(void)pUSART_Handle-> pUASRTx->USART_DR;

					//Enabling the RXE interrupt in CR1 register
					pUSART_Handle->pUASRTx->USART_CR1 |= ( 1 << USART_CR1_RXNEIE);

				}
	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             -	Configuration of IRQ interrupt in NVIC register
 *
 * @param[in]         -	IRQ Number
 * @param[in]         -	Enable or disable
 * @param[in]         -
 *
 * @return            -NONE
 *
 * @Note              -None
 */

void USART_IRQInterruptConfig(uint8_t IRQ_NUMBER,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
				if(IRQ_NUMBER <= 31)
				{
					//program ISER0 register
					*NVIC_ISER0 |= ( 1 << IRQ_NUMBER );

				}else if(IRQ_NUMBER > 31 && IRQ_NUMBER < 64 ) //32 to 63
				{
					//program ISER1 register
					*NVIC_ISER1 |= ( 1 << (IRQ_NUMBER % 32) );
				}
				else if(IRQ_NUMBER >= 64 && IRQ_NUMBER < 96 )
				{
					//program ISER2 register //64 to 95
					*NVIC_ISER3 |= ( 1 << (IRQ_NUMBER % 64) );
				}
			}else
			{
				if(IRQ_NUMBER <= 31)
				{
					//program ICER0 register
					*NVIC_ICER0 |= ( 1 << IRQ_NUMBER );
				}else if(IRQ_NUMBER > 31 && IRQ_NUMBER < 64 )
				{
					//program ICER1 register
					*NVIC_ICER1 |= ( 1 << (IRQ_NUMBER % 32) );
				}
				else if(IRQ_NUMBER >= 6 && IRQ_NUMBER < 96 )
				{
					//program ICER2 register
					*NVIC_ICER3 |= ( 1 << (IRQ_NUMBER % 64) );
				}
			}
}

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             -	Configuration of USART_IRQPriorityConfig
 *
 * @param[in]         -	IRQ Number
 * @param[in]         -	Priority
 * @param[in]         -
 *
 * @return            -NONE
 *
 * @Note              -None
 */
void USART_IRQPriorityConfig(uint8_t IRQ_NUMBER, uint8_t IRQPriority)
{

	//1. first lets find out the ipr register
		uint8_t iprx = IRQ_NUMBER / 4;
		uint8_t iprx_section  = IRQ_NUMBER %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - No_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */

void USART_IRQHandling(USART_Handle_t *pUSART_Handle)
{

	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;


/*************************Check for TC flag ********************************************/

	//Implement the code to check the state of TC bit in the SR
	temp1 = pUSART_Handle->pUASRTx->USART_SR & ( 1 << USART_SR1_TC);

	// //Implement the code to check the state of TCEIE bit
	temp2 = pUSART_Handle->pUASRTx->USART_CR1 & ( 1 << USART_CR1_TCIE);


				if ( temp1 && temp2 )
				{
					//This is due to TC flag interrupt

							//close transmission and call application callback if TxLen is zero
							if ( pUSART_Handle->TxBusyState == USART_BUSY_IN_TX)
							{
											//Check the TxLen . If it is zero then close the data transmission
											if (pUSART_Handle->TxLen ==0)
											{
												////Implement the code to clear the TC flag
												pUSART_Handle->pUASRTx->USART_SR &= ~ ( 1 << USART_SR1_TC);

												//Implement the code to clear the TCIE control bit
												pUSART_Handle->pUASRTx->USART_CR1 &= ~ ( 1 << USART_CR1_TCIE);

												//Reset the application state
												pUSART_Handle->TxBusyState = USART_READY;

												//Reset Buffer address to NULL
												pUSART_Handle->pTxBuffer = NULL;

												//Reset the length to zero
												pUSART_Handle->TxLen = 0;

												//Call the applicaton call back with event USART_EVENT_TX_CMPLT
												USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_TX_CMPLT);

											}
							}
				}


/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSART_Handle->pUASRTx->USART_SR & ( 1 << USART_SR1_TXE);

	//Implement the code to check the state of TXEIE bit in the CR1

	temp2 = pUSART_Handle->pUASRTx->USART_CR1 & ( 1 << USART_CR1_TXEIE);

			if (temp1 && temp2 )
			{
						//This is due to TXE interrupt
						if (pUSART_Handle->TxBusyState  == USART_BUSY_IN_TX)
						{
									//Keep sending data until LEN>0
									if (pUSART_Handle->TxLen >0)
									{

												//Lets Check the USART_WordLength item for 9BIT or 8BIT in a frame
												if (pUSART_Handle->pUSART_Config.USART_WordLength == USART_WORDLEN_9BITS)
												{
															//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
															pdata = (uint16_t*)pUSART_Handle->pTxBuffer;

															//Load only firsrt 9 bits of data, so we masked the value with 0x1FF
															pUSART_Handle->pUASRTx->USART_DR = (*pdata & (uint16_t) 0x1FF);


																//Check the usart parity control
																	if (pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_DISABLE)
																	{
																		//No parity is used in this transfer , so, 9bits of user data will be sent
																		//Implement the code to increment pTxBuffer twice
																		pUSART_Handle->pTxBuffer++;
																		pUSART_Handle->pTxBuffer++;
																		pUSART_Handle->TxLen -= 2;				//2bytes transfer, so - by 2

																	}
																	else
																	{
																		//Parity bit is used in this transfer . so , 8bits of user data will be sent
																		//The 9th bit will be replaced by parity bit by the hardware
																		//So increment once
																		pUSART_Handle->pTxBuffer++;
																		pUSART_Handle->TxLen -= 1;
																	}



													}else
													{
														//This is 8bit data transfer
														pUSART_Handle->pUASRTx->USART_DR = *(pUSART_Handle->pTxBuffer) & ((uint8_t)0xFF);

														//increment by once in Txbuffer
														pUSART_Handle->pTxBuffer++;
														pUSART_Handle->TxLen -=1;

													}

									}


									if (pUSART_Handle->TxLen ==0)
									{
										//TxLen is zero
										//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
										pUSART_Handle->pUASRTx->USART_CR1 &= ~( 1 << USART_CR1_TXEIE);

									}
						}
			}




/*************************Check for RXNE flag ********************************************/

		temp1 = pUSART_Handle->pUASRTx->USART_SR & ( 1 << USART_SR1_RXNE);
		temp2 = pUSART_Handle->pUASRTx->USART_CR1 & ( 1 << USART_CR1_RXNEIE);

		if (temp1 && temp2 )
		{
			//This is due to the RXNE flag( data register is full , waiting for read operation

				if (pUSART_Handle->RxBusyState == USART_BUSY_IN_RX)
				{
							//TXE is set so send data
							if (pUSART_Handle->RxLen >0)
							{
									//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
									if (pUSART_Handle->pUSART_Config.USART_WordLength == USART_WORDLEN_9BITS)
												{
																//We are going to receive 9bit data in a frame

																//Now, check are we using USART_ParityControl control or not

																if (pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_DISABLE)
																{
																			 //Then all the 9 bits are user data , so read all 9 bits

																			//read only first 9 bits so mask the DR with 0x01FF

																			*((uint16_t*)pUSART_Handle->pRxBuffer) = (pUSART_Handle->pUASRTx->USART_DR & (uint16_t)0x01FF);

																			//Increment the buffer address by twice
																			pUSART_Handle->pTxBuffer++;
																			pUSART_Handle->pTxBuffer++;

																			//Decrement the len by 2
																			pUSART_Handle->RxLen -=2;

																}
																else		//then the parity is enables
																{
																			//, so only 8 bits are user data, so mask with 0xFF
																			*(pUSART_Handle->pRxBuffer) = (pUSART_Handle->pUASRTx->USART_DR & (uint8_t)0xFF);

																			//Increment the buffer by once
																			pUSART_Handle->pRxBuffer ++;

																			//Decrement the length by once
																			pUSART_Handle->RxLen -= 1;
																}

												}else
													{	//The Word lenght 8 is selected


																//First check the Parity is enabled or not
																if (pUSART_Handle->pUSART_Config.USART_ParityControl == USART_PARITY_DISABLE)
																{
																		//Then all the 8 bits are user data
																		*(pUSART_Handle->pRxBuffer) = (uint8_t)(pUSART_Handle->pUASRTx->USART_DR &(uint8_t) 0xFF);


																}
																else
																{
																		// the parity is enabled , so only 7 bit is user data, 1 bit for parity set by hardware
																		//Read all 7 bits , except parity bit

																		*(pUSART_Handle->pRxBuffer) = (uint8_t)(pUSART_Handle->pUASRTx->USART_DR & (uint8_t)0x7F);

																}
																//Incement the buffer address by
																pUSART_Handle->pRxBuffer++;
																//Decrement the len by 1
																pUSART_Handle->RxLen -=1;
													}
							}//if >0


							if ( ! pUSART_Handle->RxLen)
							{
								//disable the rxne
								pUSART_Handle->pUASRTx->USART_CR1 &= ~(1 << USART_CR1_RXNEIE);
								//MAKe the state as ready
								pUSART_Handle->RxBusyState = USART_READY;

								//Call the application call back function
								USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_RX_CMPLT);
							}
				 }
		}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

		//the code to check the status of CTS bit in the SR
		temp1 = pUSART_Handle->pUASRTx->USART_SR & ( 1 << USART_SR1_CTS);

		//Implement the code to check the state of CTSE bit in CR3
		temp2 = pUSART_Handle->pUASRTx->USART_CR3 & ( 1 << USART_CR3_CTSE);

		//Implement the code to check the state of CTSE bit in CR3
		temp3 = pUSART_Handle->pUASRTx->USART_CR1 & ( 1 << USART_CR3_CTSIE);

		if (temp1 && temp2  && temp3)
		{
			//This is due to the CTS interrupt

			//Clearning CTS flag by writing 0 to it
			pUSART_Handle->pUASRTx->USART_SR |= ( 1 << USART_SR1_CTS);

			//Calling application call back function
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_CTS);

		}


/*************************Check for IDLE detection flag ********************************************/

		//Implement the code to check the status of IDLE flag bit in the SR
		temp1 = pUSART_Handle->pUASRTx->USART_SR &( 1 << USART_SR1_IDLE);

		temp2 = pUSART_Handle->pUASRTx->USART_CR1 & ( 1 << USART_CR1_IDLEIE);

		if ( temp1 && temp2 )
		{
			//This is due to IDLEIE flag
			temp3 = pUSART_Handle->pUASRTx->USART_SR;
			temp3 = pUSART_Handle->pUASRTx->USART_DR;

			//Calling application call back
			USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_IDLE);

		}


/*************************Check for Overrun detection flag ********************************************/

		//Implement the code to check the status of ORE flag  in the SR
		temp1 = pUSART_Handle->pUASRTx->USART_SR & ( 1 << USART_SR1_ORE);

		//Implement the code to check the status of RXNEIE  bit in the CR1
		temp2 = pUSART_Handle->pUASRTx->USART_CR1 & (1 << USART_CR1_RXNEIE);

		if (temp1 && temp2 )
		{
			//Then it is OVR error Flag

			//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

			//Calling application call back
			USART_ApplicationEventCallback(pUSART_Handle, USART_ERROR_ORE);

		}



/*************************Check for Error Flag ********************************************/

		//Noise Flag, Overrun error and Framing Error in multibuffer communication
		//We dont discuss multibuffer communication in this course. please refer to the RM
		//The blow code will get executed in only if multibuffer mode is used.

		temp2 = pUSART_Handle->pUASRTx->USART_CR3 & ( 1 << USART_CR3_EIE);

		if (temp2)
		{
					temp1 = pUSART_Handle->pUASRTx->USART_SR ;
					if ( temp1 & ( 1 << USART_SR1_FE))
					{
						//This is due to the: Framing error
									/*
										This bit is set by hardware when a de-synchronization, excessive noise or a break character
										is detected. It is cleared by a software sequence (an read to the USART_SR register
										followed by a read to the USART_DR register).
									*/
						//Calling application call back
						USART_ApplicationEventCallback(pUSART_Handle, USART_ERROR_FE);

					}

					if (temp1 & ( 1 << USART_SR1_NF))
					{
						//This is due to Noise Flag
						/*
										This bit is set by hardware when noise is detected on a received frame. It is cleared by a
										software sequence (an read to the USART_SR register followed by a read to the
										USART_DR register).
						*/
						//Calling application callback
						USART_ApplicationEventCallback(pUSART_Handle, USART_ERROR_NF);

					}

					if (temp1 & ( 1 << USART_SR1_ORE))
					{
						//This is due to the ORE flag in multibuffer communication
						/*
						 * This bit is set by hardware when the word currently being received in the shift register is
							ready to be transferred into the RDR register while RXNE=1. An interrupt is generated if
							RXNEIE=1 in the USART_CR1 register. It is cleared by a software sequence (an read to the
							USART_SR register followed by a read to the USART_DR register).
						 */

						//Calling application call back
						USART_ApplicationEventCallback(pUSART_Handle, USART_ERROR_ORE);
					}

		}

}





/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold APB clock (PCLK)
	uint32_t Pclk;

	uint32_t UsartDiv;

	//Variables to Hold Mantissa and Fractional Values
	uint32_t Mantissa_Part, Fraction_Part;

	uint32_t tempreg = 0 ;

//Get the value of APB bus clock in to the variable PCLKx (apb1 or abp2)

			if (pUSARTx == USART1  || pUSARTx == USART6)			//Both are hanging on APB1 bus
			{

				   //USART1 and USART6 are hanging on APB2 bus
					Pclk = RCC_GetPCLK2Value();
			}
			else
			{
				Pclk = RCC_GetPCLK1Value();
			}



 //Check for OVER8 configuration bit
			if (pUSARTx->USART_CR1 & ( 1 << USART_CR1_OVER8))
			{
				// Oversampling 8 is configured , OVER8 =1
				UsartDiv = ((25 * Pclk)	/ (2 * BaudRate));
			}
			else
			{
				//Oversampling 16  is configured, OVER8 = 0
				UsartDiv = ((25 * Pclk) / ( 4 * BaudRate));
			}
// Calculate the mantissa part
			Mantissa_Part = UsartDiv  / 100;

//place the mantisaa part in BBR register
			tempreg |= (Mantissa_Part << 4);

//Extract the fractional part
			Fraction_Part = UsartDiv - ( Mantissa_Part * 100);

//Calculation of final part Fractional
			if (pUSARTx->USART_CR1 & ( 1 << USART_CR1_OVER8))
			{
				//Oversampling is 8
				Fraction_Part = ( 	(	( Fraction_Part * 8) + 50 ) / 100) & ((uint8_t)0x07);			//50  is Round off
			}
			else
			{
				//Oversampling is 16
				Fraction_Part = (((Fraction_Part *16 ) + 50) / 100 ) & (uint8_t )0x0F;
			}

 //Place the fractional part in appropriate bit position . refer USART_BRR
			tempreg |= Fraction_Part;


//copy the value of tempreg in to BRR register

			pUSARTx->USART_BRR = tempreg;

}






