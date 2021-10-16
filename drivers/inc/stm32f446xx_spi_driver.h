/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Feb 14, 2021
 *      Author: Nirmal Kumar
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stm32f446xx.h>
#include "stm32f446xx_gpio_driver.h"

/**************************************************************************************************************
 * SPI configuration Structure
 */
typedef struct {

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;
}SPI_Config_t;



// Creating a Handle structure

typedef struct
{
	SPI_RegDef_t 		*pSPIx;										//Holdes the base address of SPI peripheral
	SPI_Config_t   	     SPI_Config;				 			    // Holdes the structure of SPI config structure
	uint8_t 				    *pTxBuffer;								/*<-To store the app - Tx Buffer address->*/
	uint8_t 					*pRxBuffer;								/*<-To store the app - Rx Buffer address->*/
	uint8_t 					 TxLen;										/*<-To store the Tx Len information->*/
	uint8_t					 RxLen;										/*<-To store the Rx Len information->*/
	uint8_t 					 TxState;									/*<-To store the  Tx State->*/
	uint8_t					 RxState;									/*<-To store the  Rx State->*/

}SPI_Handle_t;

/*
 * @Device Mode
 */
#define SPI_DEVICE_MODE_MASTER							1
#define SPI_DEVICE_MODE_SLAVE								0

/*
 * @Bus Config
 */
#define SPI_BUS_CONFIG_FD 									1
#define SPI_BUS_CONFIG_HD									2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			3

/*
 * @SPI Sped
 */
#define SPI_SCLK_SPEED_DIV2									0
#define SPI_SCLK_SPEED_DIV4									1
#define SPI_SCLK_SPEED_DIV8									2
#define SPI_SCLK_SPEED_DIV16									3
#define SPI_SCLK_SPEED_DIV32									4
#define SPI_SCLK_SPEED_DIV64									5
#define SPI_SCLK_SPEED_DIV128								6
#define SPI_SCLK_SPEED_DIV256								7

/*
 * @Dataframe Format
 */

#define	SPI_DFF_8BITS												0
#define  SPI_DFF_16BITS											1

/*
 * Clock Polarity (CPOL)
 */
#define SPI_CPOL_HIGH												1
#define SPI_CPOL_LOW												0

/*
 * @Clock Phase(CPHA)
 */
#define SPI_CPHA_HIGH											1
#define SPI_CPHA_LOW												0

/*
 * @SSM
 */
#define SPI_SSM_EN													1
#define SPI_SSM_DI													0



/*
 * SPI Interrput	 possible application states
 */
#define SPI_READY														0
#define SPI_BUSY_IN_RX											1
#define SPI_BUSY_IN_TX											2

/*
 * Possible API APPLICATIONS events
 */
#define SPI_EVENT_TX_CMPLT									1									//SPI event transimission complete
#define SPI_EVENT_RX_CMPLT									2									//SPI event reception complete
#define SPI_EVENT_OVR_ERR										3									//SPI event OVR error
#define SPI_EVENT_CRC_ERR										4									//SPI event CRC error


/*
 * SPI related  Status flag Definitions
 */

#define SPI_TXE_FLAG 												(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG												(1 << SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG											(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG												(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG											(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG											(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG												(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG												(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG												(1 << SPI_SR_FRE)




/********************************************************************************************************************************************
 * 																			API supported By this driver
 * 														For More information check this function definition
 * *******************************************************************************************************************************************
 */

/*
 * Peripheral Clock Setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDI);


/*
 * Init or DeInit
 */
void SPI_Init(SPI_Handle_t       *pSPI_Handle_t);
void SPI_DeInit(SPI_RegDef_t   *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t  *pSPIx  ,uint8_t *pTxBuffer , uint32_t Len);					// Base address pointer , TxBuffer pointer, how may bytes to send

void SPI_ReceiveData(SPI_RegDef_t  *pSPIx , uint8_t  *pRxBuffer , uint32_t Len );			// Base address pointer , RxBuffer pointer, how may bytes to send

/*
 * Data Send and Receive in Interrupt Mode
 */
uint8_t  SPI_SendDataIT(SPI_Handle_t *pSPI_Handle , uint8_t *pTxBuffer , uint32_t Len)	;	// Base address pointer , TxBuffer pointer, how may bytes to send

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle ,  uint8_t  *pRxBuffer , uint32_t Len );			// Base address pointer , RxBuffer pointer, how may bytes to send


/*
 * IRQ configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQnumber, uint8_t EnorDI);															//Setting the priority of the IRQ(IRQ number EnorDI)
void SPI_IRQPriorityConfig(uint8_t IRQnumber ,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPI_Handle);																			         	//IRQ handling to know ,which pin the IRQ is triggered.

/*
 * Peripheral Control API's
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t ENorDI);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx ,uint8_t ENorDI);
uint8_t SPI_Get_FlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag_Name);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTrasnmission(SPI_Handle_t *pSPI_Handle);
void SPI_CloseReception(SPI_Handle_t *pSPI_Handle);


/*
 * Applicatioin Call back
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t AppEv	);




#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
