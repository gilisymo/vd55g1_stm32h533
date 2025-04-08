/**
  ******************************************************************************
  * @file          : ls55g1_platform.h
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
#include "stm32h5xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include "stm32h5xx_util_i3c.h"


// private function prototypes

extern I3C_HandleTypeDef hi3c1;
extern void Error_Handler(void);
extern volatile uint8_t MultipleXferCplt;

I3C_CtrlTimingTypeDef CtrlTiming;
LL_I3C_CtrlBusConfTypeDef CtrlBusConf;
/* Number of Targets detected during DAA procedure */
__IO uint32_t uwTargetCount = 0;

#define TARGET1_DYN_ADDR        0x32
#define DEVICE_ID1              0U
#define Broadcast_RSTDAA        0x06
#define RXBUFFERSIZE            400
/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

typedef struct {
  char *        TARGET_NAME;          /*!< Marketing Target reference */
  uint32_t      TARGET_ID;            /*!< Target Identifier on the Bus */
  uint64_t      TARGET_BCR_DCR_PID;   /*!< Concatenation value of PID, BCR and DCR of the target */
  uint8_t       STATIC_ADDR;          /*!< Static Address of the target, value found in the datasheet of the device */
  uint8_t       DYNAMIC_ADDR;         /*!< Dynamic Address of the target preset by software/application */
} TargetDesc_TypeDef;

TargetDesc_TypeDef TargetDesc1 =
{
  "TARGET_ID1",
  DEVICE_ID1,
  0x0000000000000000,
  0x00,
  TARGET1_DYN_ADDR,
};


/* Array contain targets descriptor */
TargetDesc_TypeDef *aTargetDesc[1] = \
                          {
                            &TargetDesc1       /* DEVICE_ID1 */
                          };

I3C_XferTypeDef aContextBuffers[2];

/* Buffer used for transmission */
uint8_t aTxBuffer[0x0F];

/* Buffer used for data reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Buffer used by HAL to compute control data for the Private Communication */
uint32_t aControlBuffer[0xF];


/* Descriptor for Reset Dynamic Address Allocation (RSTDAA) broadcast CCC command */
I3C_CCCTypeDef aBroadcast_CCC[] =
{
    /*  Target Addr           CCC Value           CCC data + defbyte pointer  CCC size + defbyte         Direction        */
    {0,                Broadcast_RSTDAA,          {NULL,                  0},              LL_I3C_DIRECTION_WRITE}
};



uint8_t read_multi_i3c(uint16_t reg_index, uint8_t *pdata, uint32_t count)
{
	uint8_t read_address[2];

	uint8_t aTxBuffer1[2];
	I3C_XferTypeDef aContextBuffers1[2];
	uint32_t aControlBuffer1[2];

	read_address[0]=(reg_index >> 8) & 0xff;
	read_address[1]= reg_index & 0xff;


	I3C_PrivateTypeDef aPrivateDescriptor1[2] = \
	{
		{TARGET1_DYN_ADDR, {read_address, 2}, {NULL, 0}, HAL_I3C_DIRECTION_WRITE},
		{TARGET1_DYN_ADDR, {NULL, 0}, {pdata, count}, HAL_I3C_DIRECTION_READ}
	};


	aContextBuffers1[0].CtrlBuf.pBuffer = aControlBuffer1;
	aContextBuffers1[0].CtrlBuf.Size    = 2;
	aContextBuffers1[0].TxBuf.pBuffer   = aTxBuffer1;
	aContextBuffers1[0].TxBuf.Size      = 2;
	aContextBuffers1[0].RxBuf.pBuffer   = pdata;
	aContextBuffers1[0].RxBuf.Size      = count;

	if (HAL_I3C_AddDescToFrame(&hi3c1,NULL, &aPrivateDescriptor1[0], &aContextBuffers1[0], aContextBuffers1[0].CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK)
	{
		printf("Error - I3C Read\n");
		Error_Handler();
	}

	MultipleXferCplt = 0;
	//##- Start the multiple transfer process ###################################
	// Transmit/receive private data processus
	if (HAL_I3C_Ctrl_MultipleTransfer_IT(&hi3c1, &aContextBuffers1[0]) != HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Error_Handler();
	}

	/*##- Wait for the end of the transfer #################################*/
	/*
	 For simplicity reasons, this example is just waiting till the end of the
	 transfer, but application may perform other tasks while transfer operation
	 is ongoing. */
	while (!MultipleXferCplt)
	{
	}

	return 0;
}


int read_multi_i3c_async(I3C_HandleTypeDef *hi3c, uint16_t address, uint8_t *p_values, uint32_t size){

    int ret = 0;

    uint8_t write_address[2];
    write_address[0] = (address >> 8) & 0xFF;
    write_address[1] = address & 0xFF;

    uint32_t controlbuffer[2];
    uint32_t statusbuffer[2];
    I3C_PrivateTypeDef aPrivateDescriptor[2] = { { TARGET1_DYN_ADDR, { write_address, 2 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
                                 				 { TARGET1_DYN_ADDR, { NULL, 0 }, { p_values, size }, HAL_I3C_DIRECTION_READ } };
    I3C_XferTypeDef aContextBuffers[2] = { { { &controlbuffer[0], 1 }, { &statusbuffer[0], 1 }, { write_address, 2 }, { NULL, 0 } },
                                 			{ { &controlbuffer[1], 1 }, { &statusbuffer[1], 1 }, { NULL, 0 }, { p_values, size } } };

	if (HAL_I3C_AddDescToFrame(hi3c, NULL, &aPrivateDescriptor[0], &aContextBuffers[0],
		aContextBuffers[0].CtrlBuf.Size, I3C_PRIVATE_WITHOUT_ARB_RESTART) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_I3C_Ctrl_Transmit(hi3c, &aContextBuffers[0],100) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_I3C_GetState(hi3c) != HAL_I3C_STATE_READY)
	{
	}

	if (HAL_I3C_AddDescToFrame(hi3c, NULL, &aPrivateDescriptor[1], &aContextBuffers[1],
			aContextBuffers[1].CtrlBuf.Size, I3C_PRIVATE_WITHOUT_ARB_STOP) != HAL_OK) {
		Error_Handler();
	}
	ret = HAL_I3C_Ctrl_Receive_DMA(hi3c, &aContextBuffers[1]);
	if (ret != HAL_OK) {
		Error_Handler();
	}
	return ret;

}

uint8_t write_multi_i3c(uint16_t reg_index, const uint8_t *pdata, uint32_t count)
{
	uint8_t write_data[34];
	int i;

	write_data[0]=(reg_index >> 8) & 0xff;
	write_data[1]= reg_index & 0xff;

	if ((count> 32) || (count<1))
	{
		return 1;  // Error: count out of range
	}

	for (i = 0; i < count; i++)
    {
        write_data[i + 2] = pdata[i];
    }
	
	I3C_PrivateTypeDef aPrivateDescriptorConfig[1] = \
	{
		{TARGET1_DYN_ADDR, {write_data, count+2}, {NULL, 0}, HAL_I3C_DIRECTION_WRITE},
	};


    //##- Prepare context buffers process ##################################
    // Prepare Transmit context buffer with the different parameters
    aContextBuffers[0].CtrlBuf.pBuffer = aControlBuffer;
    aContextBuffers[0].CtrlBuf.Size    = 1;
    aContextBuffers[0].TxBuf.pBuffer   = write_data;
    aContextBuffers[0].TxBuf.Size      = count+2;

    //##- Add context buffer transmit in Frame context #####################
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &aPrivateDescriptorConfig[0], &aContextBuffers[0], aContextBuffers[0].CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_STOP) != HAL_OK)
    {
		// Error_Handler() function is called when error occurs.
		Error_Handler();
    }

    //##- Start the transmission process ###################################
    // Transmit private data processus
    if (HAL_I3C_Ctrl_Transmit_IT(&hi3c1, &aContextBuffers[0]) != HAL_OK)
    {
		/* Error_Handler() function is called when error occurs. */
		Error_Handler();
    }

    //##- Wait for the end of the transfer #################################
    //  Before starting a new communication transfer, you need to check the current
    //state of the peripheral; if it's busy you need to wait for the end of current
    //transfer before starting a new one.
    //For simplicity reasons, this example is just waiting till the end of the
    //transfer, but application may perform other tasks while transfer operation
    //is ongoing.
	while (HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY)
	{
	}
	return 0;
}

uint8_t write_multi_i3c_1byte(uint16_t reg_index, const uint8_t data)
{
	return write_multi_i3c(reg_index, &data, 1);
}

uint8_t write_multi_i3c_2bytes(uint16_t reg_index, const uint8_t data0, const uint8_t data1)
{
	uint8_t data[2] = {0x00, 0x00};
	data[0] = data0;
	data[1] = data1;
	return write_multi_i3c(reg_index, data, 2);
}

uint8_t write_multi_i3c_4bytes(uint16_t reg_index, const uint8_t data0, const uint8_t data1, const uint8_t data2, const uint8_t data3)
{
	uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
	data[0] = data0;
	data[1] = data1;
	data[2] = data2;
	data[3] = data3;
	return write_multi_i3c(reg_index, data, 4);
}


uint8_t MultiPollingValueWithMask(uint32_t numberOfPollingCycles, uint16_t address, uint8_t expectedValue, uint8_t Mask, uint32_t PoolingTimeInMs)
{

	uint8_t read_data;

	for (int i=0; i< numberOfPollingCycles; i++)
	{
		read_multi_i3c(address, &read_data, 1);

		if ((read_data & Mask) == expectedValue)
		{
			return 0;
		}
		HAL_Delay(PoolingTimeInMs); //Delay for the pooling time
	}
	return 1; //Failed to read correct value
}

uint8_t MultiPollingValue(uint32_t numberOfPollingCycles, uint16_t address, uint8_t expectedValue, uint32_t PoolingTimeInMs)
{
	return MultiPollingValueWithMask(numberOfPollingCycles, address, expectedValue, 0xFF, PoolingTimeInMs);
}

void I3C_Config(void)
{

	/*#-STEP1-#- Reset a previous Dynamic address ###############*/
	aContextBuffers[0].CtrlBuf.pBuffer = aControlBuffer;
	aContextBuffers[0].CtrlBuf.Size    = COUNTOF(aControlBuffer);
	aContextBuffers[0].TxBuf.pBuffer   = aTxBuffer;
	aContextBuffers[0].TxBuf.Size      = 1;


	/*##- Add context buffer Set CCC frame in Frame context ############################*/
	if (HAL_I3C_AddDescToFrame(&hi3c1,
					 aBroadcast_CCC,
					 NULL,
					 &aContextBuffers[0],
					 COUNTOF(aBroadcast_CCC),
					 I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Error_Handler();
	}

	if (HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c1, &aContextBuffers[0]) != HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Error_Handler();
	}

	/*##- Wait for the end of the transfer #################################*/
	while (HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY)
	{
	}

	/* #-STEP2-#- start I3C Dynamic address allocation */
	if (HAL_I3C_Ctrl_DynAddrAssign_IT(&hi3c1, I3C_ONLY_ENTDAA) != HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Error_Handler();
	}

	/*##- Wait for the end of the transfer #################################*/
	while (HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY)
	{
	}


	/* # After a dynamic address has been assigned, the sensor is recognized as an I3C device */
	/* Check if the CAM is ready to communicate in I3C */
	if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c1, TARGET1_DYN_ADDR, 300, 1000) != HAL_OK)
	{
		Error_Handler();
	}

	/* Calculate the new timing for Controller */
	CtrlTiming.clockSrcFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I3C1);
	CtrlTiming.i3cPPFreq = 1000000;
	CtrlTiming.i2cODFreq = 0;
	CtrlTiming.dutyCycle = 50;
	CtrlTiming.busType = I3C_PURE_I3C_BUS;

	/* Calculate the new timing for Controller */
	I3C_CtrlTimingComputation(&CtrlTiming, &CtrlBusConf);

	/* Update Controller Bus characteristic */
	HAL_I3C_Ctrl_BusCharacteristicConfig(&hi3c1, &CtrlBusConf);

	/* Check that device responds at 1MHz */
	if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c1, TARGET1_DYN_ADDR, 300, 1000) != HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Error_Handler();
	}

}

void SetI3CFreq(uint32_t Freq)
  {

  //change clock speed to 12.5MHz to read
          CtrlTiming.clockSrcFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I3C1);
          CtrlTiming.i3cPPFreq = Freq;
          CtrlTiming.i2cODFreq = 0;
          CtrlTiming.dutyCycle = 50;
          CtrlTiming.busType = I3C_PURE_I3C_BUS;

          /* Calculate the new timing for Controller */
          I3C_CtrlTimingComputation(&CtrlTiming, &CtrlBusConf);

          /* Update Controller Bus characteristic */
          HAL_I3C_Ctrl_BusCharacteristicConfig(&hi3c1, &CtrlBusConf);


          /* Check that device responds */
          if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c1, TARGET1_DYN_ADDR, 300, 1000) != HAL_OK)
          {
            /* Error_Handler() function is called when error occurs. */
            Error_Handler();
          }
  }

/**
  * @brief I3C target request a dynamic address callback.
  *        The main objective of this user function is to check if a target request a dynamic address.
  *        if the case we should assign a dynamic address to the target.
  * @par Called functions
  * - HAL_I3C_TgtReqDynamicAddrCallback()
  * - HAL_I3C_Ctrl_SetDynamicAddress()
  * @retval None
  */
void HAL_I3C_TgtReqDynamicAddrCallback(I3C_HandleTypeDef *hi3c, uint64_t targetPayload)
{
  /* Update Payload on aTargetDesc */
  aTargetDesc[uwTargetCount]->TARGET_BCR_DCR_PID = targetPayload;

  /* Send associated dynamic address */
  HAL_I3C_Ctrl_SetDynAddr(hi3c, aTargetDesc[uwTargetCount++]->DYNAMIC_ADDR);

}
