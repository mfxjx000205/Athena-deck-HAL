/*
 * i2c_drv.c
 * Created on: Mar 16, 2024
 * Author: Hanjie Yu
 * Project: Athena-deck I2C general driver
 * */

#include "stm32l4xx.h"
#include "stm32l4xx_ll_i2c.h"
#include "i2c_drv.h"
#include "stm32l4xx_ll_dma.h"
#include "retarget.h"

#include <stdio.h>

uint8_t i2cdevReadReg8(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg);
ErrorStatus i2cdevWriteReg8(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg , uint8_t value);
void i2cdevReadReg_Mul(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg, uint8_t size, uint8_t *rx_buffer);
uint8_t *i2cdevReadRegSeq_DMA(I2C_TypeDef *I2Cx, uint8_t SlaveAddr_IC, uint8_t target_reg, uint8_t *rx_buffer, uint16_t size, DMA_Callback callback);
ErrorStatus i2cdevWriteRegSeq_DMA();

//Function Pointer
typedef void (*DMA_Callback)(void);

//Callback Definition
DMA_Callback I2C_DMA_RX_callback = NULL;

/*
 * @brief  Read 1 byte from I2C slave's Register
 * @param  I2C_TypeDef *I2Cx  -- Number port I2C
  		   SlaveAddr_IC - Address Slave IC on bus I2C
  		   target_reg -target register need to read from
 * @retval return value read from REG
 * */

uint8_t i2cdevReadReg8(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg)
{
	uint8_t reg_value;
	//Address process
	SlaveAddr_IC = SlaveAddr_IC<<1;

	//I2C wait for : BUS BUSY
	uint32_t counter = 0;
	while(LL_I2C_IsActiveFlag_BUSY(I2Cx) == SET){
		counter++;
		if( counter == 25000 ) { //150ms
			Error_Handler();
			return 0xFF;
		}
	}

	//Write the TARGET REGISTER to the I2C slave
	LL_I2C_HandleTransfer(I2Cx , SlaveAddr_IC , LL_I2C_ADDRSLAVE_7BIT , 1 , LL_I2C_MODE_SOFTEND , LL_I2C_GENERATE_START_WRITE); //LL_I2C_GENERATE_START_READ

	//I2C wait for : TX REG BUSY
	while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);

	LL_I2C_TransmitData8(I2Cx, target_reg);

	//I2C wait for : TX REG BUSY AND CLEAR
	counter = 0;
	while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET){
		counter++;
		if( counter == 25000){ //150ms
			LL_I2C_ClearFlag_TXE(I2Cx);
			Error_Handler();
			return 0xFF;
		}
	}

	//I2C wait for : TX COMPLETE
	while(LL_I2C_IsActiveFlag_TC(I2Cx)==RESET);

	LL_I2C_HandleTransfer(I2Cx, SlaveAddr_IC, LL_I2C_ADDRSLAVE_7BIT, 1 ,LL_I2C_MODE_AUTOEND ,LL_I2C_GENERATE_START_READ);

	//I2C wait for : RX REG BUSY
	while(LL_I2C_IsActiveFlag_RXNE(I2Cx)==RESET);

	reg_value = LL_I2C_ReceiveData8(I2Cx);

	//I2C wait for : STOP DETECT
	while(LL_I2C_IsActiveFlag_STOP(I2Cx)==RESET);

	LL_I2C_ClearFlag_STOP(I2Cx);

	return reg_value;
}

/*
 * @brief  Write 1 byte to I2C slave's Register
 * @param  I2C_TypeDef *I2Cx  -- Number port I2C
  		   SlaveAddr_IC - Address Slave IC on bus I2C
  		   target_reg -target register need to write to
  		   value - value need to write to REG
 * @return return An ErrorStatus enumeration
 *         - SUCCESS:
 *         - ERROR:   Not applicable
 * */

ErrorStatus i2cdevWriteReg8(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg , uint8_t value)
{
	//Address process
	SlaveAddr_IC = SlaveAddr_IC<<1;

	//I2C wait for : BUS BUSY
	uint32_t counter = 0;
	while(LL_I2C_IsActiveFlag_BUSY(I2Cx) == SET){
		counter++;
		if( counter == 25000 ) {
			Error_Handler();
			return 0xFF;
		}
	}

	LL_I2C_HandleTransfer(I2Cx, SlaveAddr_IC, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	//I2C wait for : TX REG BUSY AND CLEAR
	while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);

    LL_I2C_TransmitData8(I2Cx, target_reg);

	//I2C wait for : TX REG BUSY AND CLEAR
    counter=0;
    while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET){
        counter++;
        if( counter == 25000 ){
            LL_I2C_ClearFlag_TXE(I2Cx);
            Error_Handler();
            return ERROR;
        }
    }

    LL_I2C_TransmitData8(I2Cx, value);

	//I2C wait for : TX REG BUSY AND CLEAR
    while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);

    //I2C wait for : STOP CLEAR
    while(LL_I2C_IsActiveFlag_STOP(I2Cx)==RESET);

    LL_I2C_ClearFlag_STOP(I2Cx);

    return SUCCESS;
}

void i2cdevReadReg_Mul(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg, uint8_t size, uint8_t *rx_buffer)
{
	//Address process
	SlaveAddr_IC = SlaveAddr_IC<<1;

	//I2C wait for : BUS BUSY
	uint32_t counter = 0;
	while(LL_I2C_IsActiveFlag_BUSY(I2Cx) == SET){
		counter++;
		if( counter == 25000 ) { //150ms
			Error_Handler();
		}
	}

	//Write the TARGET REGISTER to the I2C slave
	LL_I2C_HandleTransfer(I2Cx , SlaveAddr_IC , LL_I2C_ADDRSLAVE_7BIT , 1 , LL_I2C_MODE_SOFTEND , LL_I2C_GENERATE_START_WRITE); //LL_I2C_GENERATE_START_READ

	//I2C wait for : TX REG BUSY
	while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);

	LL_I2C_TransmitData8(I2Cx, target_reg);

	LL_I2C_ClearFlag_STOP(I2Cx);

	//I2C wait for : TX REG BUSY AND CLEAR
	counter = 0;
	while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET){
		counter++;
		if( counter == 25000){ //150ms
			LL_I2C_ClearFlag_TXE(I2Cx);
			Error_Handler();
		}
	}

	//I2C wait for : TX COMPLETE
	while(LL_I2C_IsActiveFlag_TC(I2Cx)==RESET);

//	//I2C wait for : RX REG BUSY
//	while(LL_I2C_IsActiveFlag_RXNE(I2Cx)==RESET);



	// Receive multiple bytes of data
	for (uint8_t i = 0; i < size; ++i) {
		LL_I2C_HandleTransfer(I2Cx, SlaveAddr_IC, LL_I2C_ADDRSLAVE_7BIT, 1 ,LL_I2C_MODE_SOFTEND ,LL_I2C_GENERATE_START_READ);


		rx_buffer[i] = LL_I2C_ReceiveData8(I2Cx);

		//I2C wait for : STOP DETECT

		LL_I2C_ClearFlag_STOP(I2Cx);

	}

}


uint8_t *i2cdevReadRegSeq_DMA(I2C_TypeDef *I2Cx, uint8_t SlaveAddr_IC, uint8_t target_reg, uint8_t *rx_buffer, uint16_t size, DMA_Callback callback)
{
	I2C_DMA_RX_callback = callback;
	//Address process
	SlaveAddr_IC = SlaveAddr_IC<<1;

	//I2C wait for : BUS BUSY
	uint32_t counter = 0;
	while(LL_I2C_IsActiveFlag_BUSY(I2Cx) == SET){
		counter++;
		if( counter == 25000 ) { //150ms
			Error_Handler();
			return NULL;
		}
	}

	//Write the TARGET REGISTER to the I2C slave
	LL_I2C_HandleTransfer(I2Cx , SlaveAddr_IC , LL_I2C_ADDRSLAVE_7BIT , 1 , LL_I2C_MODE_SOFTEND , LL_I2C_GENERATE_START_WRITE); //LL_I2C_GENERATE_START_READ

	//I2C wait for : TX REG BUSY
	while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET);

	LL_I2C_TransmitData8(I2Cx, target_reg);

	//I2C wait for : TX REG BUSY AND CLEAR
	counter = 0;
	while(LL_I2C_IsActiveFlag_TXE(I2Cx)==RESET){
		counter++;
		if( counter == 25000){ //150ms
			LL_I2C_ClearFlag_TXE(I2Cx);
			Error_Handler();
			return NULL;
		}
	}

	//I2C wait for : TX COMPLETE
	while(LL_I2C_IsActiveFlag_TC(I2Cx)==RESET);

	LL_I2C_ClearFlag_STOP(I2Cx);

	LL_I2C_HandleTransfer(I2Cx, SlaveAddr_IC, LL_I2C_ADDRSLAVE_7BIT, size ,LL_I2C_MODE_AUTOEND ,LL_I2C_GENERATE_START_READ);

	//I2C DMA Transfer
	LL_DMA_ConfigAddresses(DMA1,
						   LL_DMA_CHANNEL_7,
						   LL_I2C_DMA_GetRegAddr(I2C1,LL_I2C_DMA_REG_DATA_RECEIVE),
						   (uint32_t)rx_buffer,
						   LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, size);
    LL_I2C_EnableDMAReq_RX(I2C1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);
    //Debug problem :	TBD
    osDelay(1);
    //TBD: double check if need Stop clear here.
    LL_I2C_ClearFlag_STOP(I2Cx);

    return rx_buffer;
}


