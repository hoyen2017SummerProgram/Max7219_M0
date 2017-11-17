/**
  ******************************************************************************
  * @file    drv_max7219.h
  * @author  HY R&D Team
  * @version V1.0
  * @date    11/17/2017
  * @brief   This file provides low power manager data structure
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, HOYEN TECH SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2017 HOYEN TECH Co., Ltd </center></h2>
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_MAX7219_H
#define __DRV_MAX7219_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"
#include "stm32f0xx_hal.h"
  
/** @addtogroup LOW_POWER_MANAGERMENT
  * @{
  */
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

#define SPI_SS_Pin GPIO_PIN_8
#define SPI_SS_GPIO_Port GPIOA

#define CS_LOW()       HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()      HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET)

void LedControl(uint16_t numDevices);
uint8_t getDeviceCount(void);
void shutdown(uint16_t addr, bool b);
void setScanLimit(uint16_t addr, uint16_t limit);
void setIntensity(uint16_t addr, uint16_t intensity);
void clearDisplay(uint16_t addr);
void setLed(uint16_t addr, uint16_t row, uint16_t column, bool state);
void setRow(uint16_t addr, uint16_t row, uint8_t value);
void setColumn(uint16_t addr, uint16_t col, uint8_t value);
void setDigit(uint16_t addr, uint16_t digit, uint8_t value, bool dp);
void setChar(uint16_t addr, uint16_t digit, char value, bool dp);
void spiTransfer(uint16_t addr, volatile uint8_t opcode, volatile uint8_t data);
/**
  * @}
  */

#endif
