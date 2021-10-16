/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Jun 8, 2021
 *      Author: Nirmal Kumar
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"






/*
 * USART Configuration structure
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t	USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * USART Handle structure
 */

typedef struct
{
	USART_RegDef_t 		*pUASRTx;
	USART_Config_t		pUSART_Config;
	//For interrupt
	uint8_t RxLen;
	uint8_t TxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;

}USART_Handle_t;

/*
 * Usart_Modes in Interrupt
 */
#define USART_BUSY_IN_RX	    1
#define USART_BUSY_IN_TX 		2
#define USART_READY 		   	0

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX			0
#define USART_MODE_ONLY_RX			1
#define USART_MODE_TXRX				2

/*
 * USART_Application Call back
 */

#define USART_EVENT_TX_CMPLT			0
#define USART_EVENT_RX_CMPLT			1
#define USART_EVENT_CTS					2
#define USART_EVENT_IDLE				3
#define USART_ERROR_FE					4
#define USART_ERROR_NF					5
#define USART_ERROR_ORE					6

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000



/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD				2
#define USART_PARITY_EN_EVEN			1
#define USART_PARITY_DISABLE			0


/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STARTBITS_1					0
#define USART_STARTBITS_0_5					1
#define USART_STARTBITS_2					2
#define USART_STARTBITS_1_5					3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE			0
#define USART_HW_FLOW_CTRL_CTS			1
#define USART_HW_FLOW_CTRL_RTS			2
#define USART_HW_FLOW_CTRL_CTS_RTS		3

/*
 * IRQ numbers
 */
#define IRQ_NO_USART1						37
#define IRQ_NO_USART2						38
#define IRQ_NO_USART3						39
#define IRQ_NO_UART4						52
#define IRQ_NO_UART5						53




/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * USART initialization and DeInitialization
 */

void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * USART Send Data and Receive Data
 */
void USART_SendData(USART_Handle_t  *pUSART_Handle ,uint8_t *pTxBuffer,uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSART_Handle,uint8_t *pRxBuffer,uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSART_Handle,uint8_t *TxBuffer,uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSART_Handle,uint8_t *pRxBuffer,uint32_t Len);

/*
 * USART IRQ Interrupt Config and priority config
 */
void USART_IRQInterruptConfig(uint8_t IRQ_NUMBER,uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQ_NUMBER, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSART_Handle);

/*
 * USART peripheral supported API
 */

void USART_PeriClockControl(USART_RegDef_t   *pUSARTx, uint8_t EnorDi);
void USART_PeripheralControl (USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * USART flag status and clear
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint32_t FlagName);
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx,uint16_t Status_FlagName);

/*
 * USARTx Related Flags Macros
 */

#define USART_FLAG_PE				( 1 << USART_SR1_PE)
#define USART_FLAG_FE				( 1 << USART_SR1_FE)
#define USART_FLAG_NF				( 1 << USART_SR1_NF)
#define USART_FLAG_ORE				( 1 << USART_SR1_ORE)
#define USART_FLAG_IDLE				( 1 << USART_SR1_IDLE)
#define USART_FLAG_RXNE				( 1 << USART_SR1_RXNE)
#define USART_FLAG_TC				( 1 << USART_SR1_TC)
#define USART_FLAG_TXE				( 1 << USART_SR1_TXE)
#define USART_FLAG_LBD				( 1 << USART_SR1_LBD)
#define USART_FLAG_CTS				( 1 << USART_SR1_CTS)



/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
