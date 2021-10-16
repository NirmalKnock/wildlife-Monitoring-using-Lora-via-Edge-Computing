/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: May 18, 2021
 *      Author: Nirmal Kumar
 */



#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"
/*
 * Creating a Configuration Structure for 12Cx peripheral
 */

typedef struct{

	uint32_t I2C_SCLSpeed;
	uint8_t 	  I2C_DeviceAddress;
	uint8_t   I2C_ACKControl;
	uint16_t I2C_FMDuty_Cycle;

}I2C_Config_t;

/*
 * Creating Handle Structure for I2Cx Peripheral
 */

typedef struct{

	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	//For interrupt
	uint8_t *pTxBuffer;				//Tx buffer address
	uint8_t *pRxBuffer;				//Rx buffer address
	uint8_t TxLen;						//Tx len information
	uint8_t RxLen;						//Rx Length information
	uint8_t TxRxState;					//State like busy in Tx, ready, busy in Rx
	uint8_t DevAddr;					//Device address
	uint8_t RxSize;						//Rx Size
	uint8_t Sr;								//Repeated start value

}I2C_Handle_t;

/*
 * Creating Speed Macros
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		 100000
#define I2C_SCL_SPEED_FM4K	 400000
#define I2C_SCL_SPEED_FM2K	 200000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDuty_Cycle
 */
#define I2C_FM_DUTY_2			1
#define I2C_FM_DUTY_16_9		0


/********************************************************************************************************************************************
 * 																			API supported By this driver
 * 														For More information check this function definition
 * *******************************************************************************************************************************************
 */

/*
 * Peripheral Clock Setup for I2C
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDI);
void I2C_ManagaeAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * Start and stop conditions
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);




/*
 * Init or DeInit
 */
void I2C_Init(I2C_Handle_t  *pI2C_Handle_t);
void I2C_DeInit(I2C_RegDef_t   *pI2Cx);

/*
 * Master : Data Send and Receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle_t, uint8_t *pTxbuffer, uint32_t Len , uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiverData( I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint8_t len, uint8_t Slaveaddress,uint8_t Sr);


/*
 * Slave : Data Send and Receive
 */

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data);																												//We send data in byte -byte, so on need for other parameters like mastersend data
uint8_t I2C_SlaveReceiverData(I2C_RegDef_t *pI2Cx);



/*
 *  Master Data Send and Receive in Interrupt Mode
 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2C_Handle , uint8_t *pTxBuffer , uint32_t Len, uint8_t Slaveaddress,uint8_t Sr);	//Which return the application states	// Base address pointer , TxBuffer pointer, how may bytes to send
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Handle ,  uint8_t  *pRxBuffer , uint32_t Len, uint8_t Slaveaddress,uint8_t Sr );			// Base address pointer , RxBuffer pointer, how may bytes to send


/*
 * IRQ configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQnumber, uint8_t EnorDI);															//Setting the priority of the IRQ(IRQ number EnorDI)
void I2C_IRQPriorityConfig(uint8_t IRQnumber ,uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2C_Handle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2C_Handle);


/*
 * Communication closing
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle);
void I2C_CloseSendData(I2C_Handle_t  *pI2C_Handle);

/*
 * Peripheral Control API's
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi);
uint8_t I2C_Get_FlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flag_Name);
void  I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);



/*
 * Applicatioin Call back
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t AppEv	);


/*
 * I2Cx related SR1 Register Flag Macros
 */
#define I2C_FLAG_SR1_SB					(1 << I2C_SR1_SB)
#define I2C_FLAG_SR1_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_SR1_BTF				(1 << I2C_SR1_BTF)
#define I2C_FLAG_SR1_ADD10			(1 << I2C_SR1_ADD10)
#define I2C_FLAG_SR1_STOPF			(1 << I2C_SR1_STOPF)
#define I2C_FLAG_SR1_RXNE				(1 << I2C_SR1_RxNE)
#define I2C_FLAG_SR1_TXE				(1 << I2C_SR1_TxE)
#define I2C_FLAG_SR1_BERR				(1 << I2C_SR1_BERR)
#define I2C_FLAG_SR1_ARLO				(1 << I2C_SR1_ARLO)
#define I2C_FLAG_SR1_AF					(1 << I2C_SR1_AF)
#define I2C_FLAG_SR1_OVR				(1 << I2C_SR1_OVR)
#define I2C_FLAG_SR1_PECERR			(1 << I2C_SR1_PECERR)
#define I2C_FLAG_SR1_TIMEOUT		(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SR1_SMBALERT		(1 << I2C_SR1_SMBALERT)

/*
 * Macros for I2C Sr

 */
#define I2C_DISABLE_SR							0
#define I2C_ENABLE_SR								1

/*
 * I2c Interrput application states
 */
#define I2C_READY										0
#define I2C_BUSY_IN_RX							1
#define I2C_BUSY_IN_TX							2

/*
 * Possible API APPLICATIONS events
 */
#define I2C_EVENT_TX_CMPLT					0									//I2C event transimission complete
#define I2C_EVENT_RX_CMPLT					1									//I2C event reception complete
#define I2C_EV_STOP									2									//I2C event stop

#define I2C_ERROR_BERR 							 3
#define I2C_ERROR_ARLO 							 4
#define I2C_ERROR_AF   							 5
#define I2C_ERROR_OVR 						     6
#define I2C_ERROR_TIMEOUT					 7
#define I2C_EV_DATA_REQ							 8
#define I2C_EV_DATA_RCV							 9


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
