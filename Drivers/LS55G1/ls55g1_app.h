/**
  ******************************************************************************
  * @file          : ls55g1_driver.h
  * @author        : Giovanni@Gilisymo
  * @brief         : This file provides header code for the demo application.
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
#ifndef LS55G1_LS55G1_APP_H_
#define LS55G1_LS55G1_APP_H_

#include <sys/types.h>
#include <string.h> // Include for memset
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "ls55g1_platform.h"
#include "ls55g1_driver.h"

#ifndef UART_COMM_DEBUG
#define Printf_Warn(msg, ...)  printf( "Warning: " msg"\n", ##__VA_ARGS__)
#define Printf_Error(msg, ...)  printf( "Error: " msg"\n", ##__VA_ARGS__)
#define Printf_Debug(msg, ...)  printf("Debug: " msg"\n", ##__VA_ARGS__)
#else
#   define Printf_Warn( ...)  (void)0
#   define Printf_Error( ...)  (void)0
#   define Printf_Debug( ...)  (void)0
#endif


void ls55g1_setup(void);

void ls55g1_main(void);


#endif /* LS55G1_LS55G1_APP_H_ */
