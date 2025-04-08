/**
  ******************************************************************************
  * @file          : ls55g1_driver.h
  * @author        : Giovanni@Gilisymo
  * @brief         : This file provides code for communication with i3c.
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2025 Gilisymo.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef INC_LS55G1_PLATFORM_H_
#define INC_LS55G1_PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"
#include <stdio.h>
#include <sys/types.h>

uint8_t read_multi_i3c(uint16_t reg_index, uint8_t *pdata, uint32_t count);
uint8_t write_multi_i3c(uint16_t reg_index, const uint8_t *pdata, uint32_t count);
uint8_t write_multi_i3c_1byte(uint16_t reg_index, const uint8_t data);
uint8_t write_multi_i3c_2bytes(uint16_t reg_index, const uint8_t data0, const uint8_t data1);
uint8_t write_multi_i3c_4bytes(uint16_t reg_index, const uint8_t data0, const uint8_t data1, const uint8_t data2, const uint8_t data3);

uint8_t MultiPollingValue(uint32_t numberOfPollingCycles, uint16_t address, uint8_t expectedValue, uint32_t PoolingTimeInMs);
uint8_t MultiPollingValueWithMask(uint32_t numberOfPollingCycles, uint16_t address, uint8_t expectedValue, uint8_t Mask, uint32_t PoolingTimeInMs);
void I3C_Config(void);
void SetI3CFreq(uint32_t Freq);


int read_multi_i3c_async(I3C_HandleTypeDef *hi3c, uint16_t address, volatile uint8_t *p_values, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* INC_LS55G1_PLATFORM_H_ */
