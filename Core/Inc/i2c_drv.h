/*
 * i2c_drv.h
 *
 *  Created on: Mar 16, 2024
 *      Author: mfxjx
 */

#ifndef INC_I2C_DRV_H_
#define INC_I2C_DRV_H_

typedef void (*DMA_Callback)(void);

uint8_t i2cdevReadReg8(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg);
ErrorStatus i2cdevWriteReg8(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg , uint8_t value);
void i2cdevReadReg_Mul(I2C_TypeDef *I2Cx , uint8_t SlaveAddr_IC , uint8_t target_reg, uint8_t size, uint8_t *rx_buffer);
uint8_t *i2cdevReadRegSeq_DMA(I2C_TypeDef *I2Cx, uint8_t SlaveAddr_IC, uint8_t target_reg, uint8_t *rx_buffer, uint16_t size, DMA_Callback callback);
ErrorStatus i2cdevWriteRegSeq_DMA();

#endif /* INC_I2C_DRV_H_ */
