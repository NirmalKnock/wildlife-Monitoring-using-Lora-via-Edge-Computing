/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Feb 14, 2021
 *      Author: Nirmal Kumar
 */

#include  "stm32f446xx_spi_driver.h"

//SPI userfuction prototypes//

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);

/*stm32f446xx_spi_driver.c */
/*
 * Peripheral Clock configuration
 */

/************************************************************************************************
 *
  * @fn					- SPI_PeriClockControl
  *
  * @brief				- Which controls the GPIO clock Enable or Disable for the given GPIO port.
  *
  *	@param[in]			-  Address of RegDef_t Structure
  *	@param[in]			-  Enable or Disable value(MACROS)
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (pSPIx==SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx==SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx ==SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if  (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
   	}


}

/*
 * SPI Init and Deint
 */

/************************************************************************************************
 *
  * @fn					- SPI_Init
  *
  * @brief				- Which Initialize the SPI peripheral
  *
  *	@param[in]			-  Address of Handle Structure and Address of SPI_RegDef_t Structure
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void SPI_Init (SPI_Handle_t *SPI_Handle_t)
{
	SPI_PeriClockControl(SPI_Handle_t->pSPIx, ENABLE);
	//first configure the CR1 register
	uint32_t tempreg = 0;

	//1. Configure the Device Mode
	tempreg  |= (SPI_Handle_t->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR );



	//2. Bus configuration

	if (SPI_Handle_t->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD )
	{
		// Clear the BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE ); 		// Fullduplex

	}
	else if (SPI_Handle_t->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode shoud be set
		tempreg |= (1<< SPI_CR1_BIDIMODE );  			// Halfduplex
	}
	else if (SPI_Handle_t->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI Mode should be cleared
		tempreg  &= ~(1<< SPI_CR1_BIDIMODE );
		// and RX only only is SET
		tempreg   |=    (1<< SPI_CR1_RXONLY);
	}

	//3. SPI speed Configuration

	tempreg |= (SPI_Handle_t->SPI_Config.SPI_Speed  << SPI_CR1_BR);

	//4. Data frame format
	tempreg |= (SPI_Handle_t->SPI_Config.SPI_DFF << SPI_CR1_DFF	);

	//5. Clock Polarity (CPOL)
	tempreg |= (SPI_Handle_t->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	//6. Clock Phase (CPHA)
	tempreg |= (SPI_Handle_t->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);


	//7.SSM Mode
	tempreg |= (SPI_Handle_t->SPI_Config.SPI_SSM <<SPI_CR1_SSM);

	SPI_Handle_t->pSPIx->CR1 = tempreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}

}

uint8_t  SPI_Get_FlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag_Name)
{
	if (pSPIx->SR & Flag_Name)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 *Data Read and Write
 */

/************************************************************************************************
 *
  * @fn					- SPI_Send_Data
  *
  * @brief				- Which send the data
  *
  *	@param[in]			-  SPI register base address
  *	@param[in]			-  pointer address of TxBuffer
  *	@param[in]			- Length of the data
  *
  *	return				- none
  *
  *	@Note				- This is Blocking call

*/

void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t	 *pTxBuffer , uint32_t Len)
{
	while (Len > 0){

		//1. Wait untill Tx Buffer is Zero
		while(SPI_Get_FlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);   // wait empty in Tx buffer .from Status register

		//2. Check the DFF Bit in CR1 register
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//then it is 16 bit data format
			//1. Load the data in Data register
			pSPIx->DR  = *(( uint16_t*) pTxBuffer)   ;     // typecast to uint16 ,and make this pointer
			Len--;
			Len--;   // two times because it is 16 bit data
		  (uint16_t*) pTxBuffer++ ;    							// Increment for next data

		}
		else
		{
			//it willbe 8 bit data format
			pSPIx ->DR = *pTxBuffer ;
			Len --;
			pTxBuffer++ ;
		}

	}

}

/*
 *Data Read
 */

/************************************************************************************************
 *
  * @fn					- SPI_Read_Data
  *
  * @brief				- Which Reads the data
  *
  *	@param[in]			-  SPI Register base address
  *	@param[in]			-  * Pointer address of RxBuffer
  *	@param[in]			-  Length of the data
  *
  *	return				- none
  *
  *	@Note				- none

*/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer , uint32_t Len)
{
	while (Len >0)
	{
		//1. Wait untill RXNE is SET
		while (SPI_Get_FlagStatus(pSPIx, SPI_RXNE_FLAG) ==  FLAG_RESET);

		//2. Read from the DFF bit in DR
		if ( pSPIx->CR1  & ( 11 << SPI_CR1_DFF))
		{
			//16bit data
			//Load data from DR to Rx Buffer address
			*((uint16_t*)pRxBuffer)  = pSPIx->DR ;

			// Decrement the Len
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++; 		// to point to next free memory
		}
		else
		{
			// 8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer ++;
		}
	}
}

/************************************************************************************************
 *
  * @fn					- SPI_Read_DataIT
  *
  * @brief				- Which Reads the data at Interrupt Mode
  *
  *	@param[in]			-  SPI Handle Structure Pointer
  *	@param[in]			-  * Pointer address of RxBuffer
  *	@param[in]			-  Length of the data
  *
  *	return				- it returns the State
  *
  *	@Note				- none

*/

uint8_t  SPI_SendDataIT(SPI_Handle_t *pSPI_Handle , uint8_t *pTxBuffer , uint32_t Len)
{
	//The code will run , if the peripheral is free
	uint8_t State = pSPI_Handle->TxState;

	if (State != SPI_BUSY_IN_TX)
	{
	//1. Save the TX buffer addres  and Len in some Global Variable
	pSPI_Handle->pTxBuffer = pTxBuffer;

	//Save the TXlength
	pSPI_Handle->TxLen = Len;

	//2. Mark the SPI state has Busy in Transmission
	//So, no other code can take over the SPI peripheral untill the transmission is over
	pSPI_Handle->TxState = SPI_BUSY_IN_TX;

	//3.Enable the TXEIE Control Bit to get an Interrupt, When ever the TXE flag is set in SR register
	pSPI_Handle->pSPIx->CR2 |= (1 <<  SPI_CR2_TXEIE );

	//4. Data Transmission will be handled by ISR code (imlpement later)

	}
	return State;

}

/************************************************************************************************
 *
  * @fn					- SPI_ReceiveDataIT
  *
  * @brief				- Which Receive the data in Interrupt Mode
  *
  *	@param[in]			-  SPI Handle Structure pointer
  *	@param[in]			-  * Pointer address of RxBuffer
  *	@param[in]			-  Length of the data
  *
  *	return				- none
  *
  *	@Note				- none

*/
uint8_t  SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle ,  uint8_t  *pRxBuffer , uint32_t Len )

{
	//The code run, if its free
	uint8_t state = pSPI_Handle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. Store the RX buffer address and len information in some global variables
		pSPI_Handle->pRxBuffer = pRxBuffer;
		pSPI_Handle->RxLen = Len;

		//2. Mark the SPI stare has busy in transmission so no other code can't take over the peripheral
		pSPI_Handle->RxState = SPI_BUSY_IN_RX;

		//Enable the RXNE control bit to get a interrupt whenever the RXNE flag is 	Set in SR
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}
	return state;
}


/************************************************************************************************
 *
  * @fn					- SPI_IRQInterrupt Config
  *
  * @brief				- Which Configure the Interrupt in NVIC register
  *
  *	@param[in]			-  IRQ Number
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/


void SPI_IRQInterruptConfig(uint8_t IRQnumber, uint8_t EnorDI)
{
	if (EnorDI == ENABLE)
	{
		if (IRQnumber <=31 )
		{
			*NVIC_ISER0 |= (1 << IRQnumber);
		}
		else if ( IRQnumber >31 && IRQnumber < 64)				//32 to 63
		{
			*NVIC_ISER1 |= ( 1 << (IRQnumber%32));
		}
		else if (IRQnumber >= 64 && IRQnumber < 95)				//64 to 95 Program ICER3 register in NVIC
		{
			*NVIC_ISER2 |= ( 1 << (IRQnumber %64));
		}
	}

	else
	{
		//Program to clear in INTERRUPT CLEAER ENABLE REGISTER
		if (IRQnumber <31)
		{
			*NVIC_ICER0 |= ( 1 << IRQnumber);
		}
		else if (IRQnumber >=31 && IRQnumber <64)
		{
			*NVIC_ICER1 |= ( 1 << (IRQnumber % 32));				//from 32  to 63
		}
		else if (IRQnumber >=64 && IRQnumber < 96)
		{
			*NVIC_ICER2 |= ( 1 << (IRQnumber %64));
		}
	}
}


/************************************************************************************************
 *
  * @fn					- SPI_IRQPriorityConfig
  *
  * @brief				- Which sets the priority
  *
  *	@param[in]			-  SPI Register base address
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
void SPI_IRQPriorityConfig(uint8_t IRQnumber ,uint8_t IRQPriority)
{
	uint8_t ipr_section;
	uint8_t ipr_bitfield;
	uint8_t shift_amount;
	ipr_section = IRQnumber / 4;  				//check notes for formula
	ipr_bitfield = IRQnumber %4;

	shift_amount = (8 * ipr_bitfield) + ( 8 - No_PR_BITS_IMPLEMENTED);				//Because in some MCU have reserved

	*(NVIC_PR_BASE_ADDR + ipr_section ) |= ( IRQnumber <<  shift_amount);


}





/************************************************************************************************
 *
  * @fn					- SPI_IRQHanfling
  *
  * @brief				- Which Handles the IRQ
  *
  *	@param[in]			-  SPI Register base address
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle)
{

	uint8_t temp1, temp2;
	// 1.Lets first check for TXE
	temp1 = pSPI_Handle->pSPIx->SR & (1 <<SPI_SR_TXE);
	//Lets check the TXIE Bit
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	//if both temp1 and temp 2 are 1 means the interrupt from TXE
	if(temp1 && temp2)
	{
		// 	Handle TXE
		spi_txe_interrupt_handle(pSPI_Handle);
	}

	//2.Lets check for RXE

	temp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_RXNE);
	//Lets check the RXNIE
	temp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2 )
	{
		spi_rxne_interrupt_handle(pSPI_Handle);
	}

	//Check for OVR flag
	 temp1= pSPI_Handle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	 temp2 = pSPI_Handle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	 if (temp1 & temp2)
	 {
		 spi_ovr_err_interrupt_handle(pSPI_Handle);
	 }


}

/************************************************************************************************
 *
  * @fn					- SPI_PeripheralControl
  *
  * @brief				- Which Enable the SPI peripheral
  *
  *	@param[in]			-  SPI Register base address
  *	@param[in]			- Enable or Disable
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SPE);
	}
}

/************************************************************************************************
 *
  * @fn					- SPI_SSOEConfig
  *
  * @brief				- Which Enable the SSOE ,when SPE =1(Enabled)
  *
  *	@param[in]			-  SPI Register base address
  *	@param[in]			- Enable or Disable
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx , uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}

/************************************************************************************************
 *
  * @fn					- SPI_SSIConfig
  *
  * @brief				- Which Enable or Disables the SSI bit
  *
  *	@param[in]			-  SPI Register base address
  *	@param[in]			- Enable or Disable
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/

void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/************************************************************************************************
 *
  * @fn					- spi_txe_interrupt_handle
  *
  * @brief				- Which Handle the TXE interrupt
  *
  *	@param[in]			-  SPI handle pointer
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none

*/
// helper function prototypes

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	//Check the Frame format  16bit or 8bit
	if((pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF ) ) )
	{
		//DFF is 1 ,ie 16bit
		//Load the data into TXbuffer
		pSPI_Handle->pSPIx->DR =  *((uint16_t*)pSPI_Handle->pTxBuffer) ;
		pSPI_Handle->TxLen--;
		pSPI_Handle->TxLen--;
		(uint16_t*)pSPI_Handle->pTxBuffer++;
	}
	else
	{
		pSPI_Handle->pSPIx->DR =  *pSPI_Handle->pTxBuffer ;
		pSPI_Handle->TxLen--;
		pSPI_Handle->pTxBuffer++;
	}

	if (! pSPI_Handle->TxLen )
	{
		SPI_CloseTrasnmission(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_TX_CMPLT);
	}
}
/************************************************************************************************
 *
  * @fn					- spi_rxne_interrupt_handle
  *
  * @brief				- Which handle the RXNE interrupt
  *
  *	@param[in]			-  SPI handle Pinter
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none
*/

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	//1. First check DFF is 16 bit or 8bit
	if ( pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		// It is 16Bit DFF
		//Read the data from Rx Buffer
		*((uint16_t*) pSPI_Handle->pRxBuffer) = (uint16_t) pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen = 2;
		pSPI_Handle-> RxLen --;
		pSPI_Handle->RxLen --;
		(uint16_t*)pSPI_Handle->pRxBuffer ++;
	}
	else
	{
		//8 bit
		*(pSPI_Handle->pRxBuffer) = (uint8_t) pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen --;
		pSPI_Handle->pRxBuffer ++;

	}
	//Closing the Reception
	if (!pSPI_Handle->RxLen)
	{
		 SPI_CloseReception(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_RX_CMPLT);

	}

}


/************************************************************************************************
 *
  * @fn					- SPI_CloseTrasnmission
  *
  * @brief				- which close the SPI transmission
  *
  *	@param[in]			-  SPI handle Pointer
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none
*/
void SPI_CloseTrasnmission(SPI_Handle_t *pSPI_Handle)
{
	//Txlen is zero ,so close the SPI communication and inform the application TX is over
			//So clear the RXNEIE bit
			//This prevents Interrupt from RXNE flag
			pSPI_Handle->pSPIx->CR2 &= ~ (1 << SPI_CR2_RXNEIE);

			//Resetting the Rx Buffer
			pSPI_Handle->pRxBuffer =NULL;
			pSPI_Handle->RxLen = 0;
			pSPI_Handle->RxState = SPI_READY;
}
/***********************************************************************************************
 *
  * @fn					- SPI_CloseReception
  *
  * @brief				- Which closes the SPI reception
  *
  *	@param[in]			-  SPI handle Pinter
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none
*/
void SPI_CloseReception(SPI_Handle_t *pSPI_Handle)
{
	//Reception is complete
	//Lets 	turn off the rxneie interrupt
	pSPI_Handle->pSPIx->CR2 &= ~ (1 << SPI_CR2_RXNEIE);
	pSPI_Handle->pRxBuffer = NULL;
	pSPI_Handle->RxLen = 0;
	pSPI_Handle->RxState = SPI_READY;
}


/************************************************************************************************/

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEv	)
{
	//This is the Weak Implementation Application May overwrite this Function

}

/************************************************************************************************
 *
  * @fn					- SPI_ClearOVRFlag
  *
  * @brief				- which handles the OVR flag clearing process
  *
  *	@param[in]			-  SPI handle Pointer
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none
*/

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


/************************************************************************************************
 *
  * @fn					- spi_ovr_err_interrupt_handle
  *
  * @brief				- Which handle OVR flag error
  *
  *	@param[in]			- SPI handle structure
  *	@param[in]			-
  *	@param[in]			-
  *
  *	return				- none
  *
  *	@Note				- none
*/
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	uint8_t temp;
	//eg at the time of transmission some other can transmit , so the code executes if only the SPI is not is transmission.
	//1.clear the OVR flag by using the dummy variable
	if (pSPI_Handle->TxState != SPI_BUSY_IN_TX)
	{
	temp = pSPI_Handle->pSPIx->DR;			//Reading access(datasheet)
	temp = pSPI_Handle->pSPIx->SR;			//Reading access(datasheet)

	}
	(void)temp;
	//2. And inform the application
	SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_OVR_ERR);

}
