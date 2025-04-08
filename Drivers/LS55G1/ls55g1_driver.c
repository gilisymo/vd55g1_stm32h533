/**
  ******************************************************************************
  * @file          : ls55g1_driver.c
  * @author        : Giovanni@Gilisymo
  * @brief         : This file provides code to drive the VD55G1.
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

#include <ls55g1_platform.h>
#include "ls55g1_driver.h"

struct Config_t Config =
{
	.AutoGain =					1,
	.Binning2x2 =				0,
	.Exposure =					750,
	.AnalogGain =				1.00,
	.FrameLength = 				800,
	.Sensor_X_Start = 			200,
	.Sensor_Y_Start = 			174,
	.DigitalGain = 				1.1,
};

void ls55g1_init_cut20(void)
{
	uint8_t ReturnValue = 0;

	uint16_t address = START_PATCH_ADDRESS;
	// ===================
	// Power up the device
	// ===================

	// Wait sensor to wake up
	MultiPollingValue(5, UI_STATUS_DEVICE_MODEL_ID+3, 0x53, 1); // (val=1396000561) / UI.STATUS.DEVICE_MODEL_ID
	MultiPollingValue(5, UI_STATUS_DEVICE_MODEL_ID+2, 0x35, 1); // (val=1396000561) / UI.STATUS.DEVICE_MODEL_ID
	MultiPollingValue(5, UI_STATUS_DEVICE_MODEL_ID+1, 0x47, 1); // (val=1396000561) / UI.STATUS.DEVICE_MODEL_ID
	MultiPollingValue(5, UI_STATUS_DEVICE_MODEL_ID, 0x31, 1); // (val=1396000561) / UI.STATUS.DEVICE_MODEL_ID

	// Wait FSM to be READY_TO_BOOT
	HAL_Delay(2);
	MultiPollingValue(5, VD55G1_SYSTEM_FSM, 0x1, 0); // (val=1) UI.STATUS.SYSTEM_FSM.VALUE


	// ===========================
	// Operations in READY_TO_BOOT
	// ===========================

	// Load patch (2.0)

	const uint8_t patchData1a[25][30] = {
		{0xb2, 0x0, 0x0, 0x2, 0x52, 0x0, 0x42, 0x0, 0x56, 0x0, 0x42, 0x0, 0x5c, 0x0, 0x42, 0x0, 0x52, 0x0, 0x42, 0x0, 0x4c, 0x4, 0x4, 0xfa, 0xc6, 0xf, 0x94, 0xe0, 0x19, 0xe},
		{0xc9, 0x65, 0x1, 0xc0, 0x28, 0xde, 0xa, 0x42, 0x80, 0xe0, 0x20, 0x42, 0xf8, 0xf3, 0x16, 0xde, 0xc5, 0x8a, 0x19, 0x0, 0xb8, 0xe0, 0x10, 0x2, 0xc, 0xec, 0x1d, 0xe6, 0x14, 0x2},
		{0x88, 0x80, 0x4e, 0x4, 0x1, 0x0, 0x10, 0x80, 0x25, 0x2, 0x8, 0x9c, 0x86, 0x2, 0x0, 0x80, 0x8, 0x44, 0x0, 0x98, 0x55, 0x81, 0x11, 0x85, 0x45, 0x81, 0x11, 0x89, 0x25, 0x81},
		{0x11, 0x83, 0x2b, 0x0, 0x24, 0xe0, 0x64, 0xc2, 0xb, 0x84, 0xa8, 0x5d, 0x0, 0xef, 0x2b, 0x80, 0x1, 0x83, 0x1b, 0x8c, 0xd8, 0x49, 0x60, 0xef, 0xb, 0xa1, 0x65, 0x82, 0xb, 0xe},
		{0x88, 0xf9, 0xa, 0x0, 0x0, 0xe8, 0x9, 0xe, 0xc, 0x80, 0x0, 0x40, 0x4, 0x9c, 0x1, 0x4e, 0xc, 0x80, 0x4c, 0xc, 0x4, 0xf2, 0x93, 0xdd, 0x4c, 0x4, 0xc, 0xfe, 0x46, 0x4f},
		{0xfc, 0xe0, 0x6b, 0x80, 0x84, 0x9c, 0x46, 0x41, 0x38, 0xe2, 0xe, 0x9c, 0xb, 0x8c, 0xe8, 0x4a, 0x60, 0xef, 0xb, 0x8c, 0x8e, 0x9c, 0x28, 0x4f, 0x60, 0xef, 0xb, 0xa1, 0x6a, 0x40},
		{0x80, 0xe0, 0x85, 0x86, 0x4a, 0x4c, 0x80, 0xe0, 0x25, 0x86, 0x78, 0x57, 0x60, 0xef, 0x96, 0x4d, 0x9c, 0xe1, 0x1, 0x81, 0x6, 0x98, 0x76, 0xe, 0xe8, 0xe0, 0xb5, 0x81, 0x8, 0x9c},
		{0x4a, 0x40, 0x88, 0xe0, 0x85, 0x80, 0x8, 0x41, 0x0, 0xe8, 0x35, 0x81, 0x18, 0x41, 0x0, 0xe8, 0x8, 0x98, 0x4a, 0x0, 0xfc, 0xfb, 0x95, 0xfc, 0x38, 0x59, 0x60, 0xef, 0x2c, 0x0},
		{0x0, 0xe2, 0x4c, 0xc, 0xc, 0xf6, 0x93, 0xdd, 0x83, 0xc1, 0x13, 0xc5, 0x93, 0xdd, 0xc3, 0xc1, 0x83, 0xc1, 0x13, 0xc3, 0x93, 0xdd, 0xc3, 0xc1, 0xd, 0xc3, 0x1a, 0x41, 0x8, 0xe4},
		{0xa, 0x40, 0x84, 0xe1, 0xc, 0x0, 0x0, 0xe2, 0x93, 0xdd, 0x4c, 0x4, 0x4, 0xfa, 0x86, 0x4e, 0xec, 0xe1, 0x8, 0x9e, 0x65, 0xe, 0x24, 0xf8, 0xe, 0x2, 0x99, 0x7a, 0x0, 0xc0},
		{0x0, 0x40, 0xf8, 0xf3, 0x6, 0x9e, 0xb, 0x8c, 0x28, 0x57, 0x0, 0xef, 0x25, 0xe, 0x28, 0xf8, 0x2, 0x2, 0xfc, 0xed, 0xf6, 0x49, 0xfd, 0x6f, 0xe0, 0xff, 0x4, 0xca, 0x14, 0x6},
		{0xc0, 0xe0, 0xf, 0x88, 0x3f, 0xa0, 0xb, 0x8c, 0x3e, 0xca, 0x28, 0x56, 0x0, 0xef, 0x86, 0x2, 0x84, 0xfe, 0xe, 0x5, 0x9, 0x7d, 0x0, 0xc0, 0x5, 0x4e, 0x8, 0xf8, 0x18, 0x7d},
		{0xfc, 0xef, 0x4a, 0x40, 0x80, 0xe0, 0x9, 0xe, 0x4, 0xc0, 0x0, 0x40, 0x40, 0xdc, 0x1, 0x4e, 0x4, 0xc0, 0x4c, 0xc, 0x4, 0xf2, 0x93, 0xdd, 0xc, 0x0, 0x80, 0xfa, 0x15, 0x0},
		{0x3c, 0xe0, 0x21, 0x81, 0x31, 0x85, 0x21, 0x42, 0x60, 0xe0, 0x15, 0x0, 0x44, 0xe0, 0x31, 0x42, 0x40, 0xe1, 0x15, 0x0, 0x34, 0xe0, 0x21, 0x42, 0x20, 0xe0, 0x15, 0x0, 0x34, 0xe0},
		{0xd6, 0x4, 0x10, 0xe0, 0x23, 0x42, 0x30, 0xe0, 0x15, 0x0, 0x34, 0xe0, 0x86, 0x44, 0x4, 0xe0, 0x23, 0x42, 0x38, 0xe0, 0x5, 0x0, 0x30, 0xe0, 0xc6, 0x2, 0x8, 0xe0, 0x13, 0x40},
		{0x10, 0xe3, 0x88, 0x75, 0x40, 0xef, 0x6, 0x40, 0xc, 0xe1, 0x4, 0x80, 0x6, 0x2, 0x94, 0xe0, 0x2b, 0x2, 0xc4, 0xea, 0x3b, 0x0, 0x78, 0xe2, 0x20, 0x44, 0xfd, 0x73, 0x7, 0xc0},
		{0x30, 0x46, 0x1, 0x70, 0xf8, 0xc0, 0x3f, 0xa4, 0x33, 0x40, 0x78, 0xe2, 0xa, 0x84, 0xc, 0x8, 0x80, 0xf2, 0x98, 0x1a, 0x40, 0xff, 0xc3, 0xc1, 0x6, 0x40, 0xc, 0xe1, 0x4, 0x80},
		{0x1b, 0x0, 0x40, 0xe4, 0x19, 0xc2, 0x13, 0x40, 0x40, 0xe4, 0x1b, 0x0, 0x40, 0xe4, 0x19, 0xc4, 0x13, 0x40, 0x40, 0xe4, 0x93, 0xdd, 0xc6, 0x43, 0xec, 0xe0, 0x46, 0x41, 0xfc, 0xe0},
		{0x24, 0x84, 0x4, 0x80, 0x31, 0x81, 0x4a, 0x44, 0x80, 0xe0, 0x86, 0x44, 0xc, 0xe1, 0x9, 0x0, 0x6c, 0xe0, 0xc4, 0x8a, 0x8e, 0x47, 0xfc, 0x9f, 0x1, 0x42, 0x51, 0x78, 0xc, 0xc0},
		{0x31, 0x58, 0x90, 0xe0, 0x34, 0x8a, 0x41, 0xbf, 0x6, 0x8, 0x0, 0xc0, 0x41, 0x46, 0xa0, 0xe0, 0x34, 0x8a, 0x51, 0x81, 0xf6, 0xb, 0x0, 0xc0, 0x51, 0x46, 0xd0, 0xe0, 0x34, 0x8a},
		{0x1, 0xbf, 0x51, 0x46, 0xe0, 0xe0, 0x44, 0x84, 0xa, 0x48, 0x84, 0xe0, 0x75, 0x86, 0x54, 0xca, 0x49, 0x88, 0x44, 0x6, 0x88, 0xe1, 0x36, 0x94, 0x4a, 0x46, 0x80, 0xe0, 0x34, 0xca},
		{0x47, 0xc6, 0x11, 0x8d, 0x41, 0x46, 0xd0, 0xe0, 0x34, 0x88, 0x76, 0x2, 0x0, 0xc0, 0x6, 0x0, 0x0, 0xc0, 0x16, 0x8c, 0x14, 0x88, 0x1, 0x42, 0xc0, 0xe1, 0x1, 0x42, 0xe0, 0xe1},
		{0x1, 0x42, 0xf0, 0xe1, 0x93, 0xdd, 0x34, 0xca, 0x41, 0x85, 0x46, 0x8c, 0x34, 0xca, 0x6, 0x48, 0x0, 0xe0, 0x41, 0x46, 0xd0, 0xe0, 0x34, 0x88, 0x41, 0x83, 0x46, 0x8c, 0x34, 0x88},
		{0x1, 0x46, 0xc0, 0xe1, 0x1, 0x46, 0xe0, 0xe1, 0x1, 0x46, 0xf0, 0xe1, 0x9, 0x2, 0x20, 0xe0, 0x14, 0xca, 0x3, 0x42, 0x58, 0xe0, 0x93, 0xdd, 0x0, 0x0, 0x6, 0xa9, 0x0, 0x0},
		{0x74, 0xff, 0x40, 0x0, 0x8, 0x5, 0x80, 0xe0, 0xa8, 0xc1, 0x40, 0x0, 0x48, 0x37, 0x9c, 0xe0, 0x40, 0x6c, 0x40, 0x0, 0xc8, 0x26, 0xc8, 0xe0, 0xfc, 0x91, 0x40, 0x0, 0xc8, 0x0}

	};

	for (int i = 0; i < 25; i++) {
		write_multi_i3c(address, &patchData1a[i][0], 30);
		address += 30;
	}

	const uint8_t patchData1b[26] = {0xb8, 0xe0, 0x30, 0x16, 0x41, 0x0, 0x88, 0x1a, 0x74, 0xe0, 0xb0, 0x7e, 0x40, 0x0, 0x48, 0x1a, 0xc0, 0xe0, 0x0, 0xb9, 0x0, 0x0, 0xf9, 0xd9, 0x0, 0x0};

	write_multi_i3c(address, &patchData1b[0], 26);

	// Run command PATCH_AND_BOOT
	write_multi_i3c_1byte(UI_CMD_BOOT_COMMAND, 0x2); // (val=2) / UI.CMD.BOOT.COMMAND
	HAL_Delay(200);
	ReturnValue = MultiPollingValueWithMask(18, 0x0200, 0x0, 0x3, 1); // (val=0) UI.CMD.BOOT.COMMAND
	printf("Run Patch and Boot: %d\n",ReturnValue);

	// Wait FSM to be SW_STBY
	HAL_Delay(200);
	ReturnValue = MultiPollingValue(5, VD55G1_SYSTEM_FSM, 0x2, 0); // (val=2) UI.STATUS.SYSTEM_FSM.VALUE
	printf("Wait FSM to be in SW Standby: %d\n",ReturnValue);

	// Run command START_VTRAM_UPDATE
	write_multi_i3c_1byte(VD55G1_CMD_STBY, 0x6);
	HAL_Delay(200);
	ReturnValue = MultiPollingValueWithMask(4, VD55G1_CMD_STBY, 0x0, 0xf, 1); // (val=0) UI.CMD.STBY.COMMAND
	printf("Run command START_VTRAM_UPDATE: %d\n",ReturnValue);

	write_multi_i3c_2bytes(0xc638, 0x21, 0x0); // 0xc638

	
	const uint8_t patchData2a[10][30] = {
		{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x60, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x20, 0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0, 0xc, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x1c, 0x20, 0x0},
		{0x0, 0x0, 0x0, 0x0, 0xa, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x42, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x1c, 0x20, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1c},
		{0x20, 0x4, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1c, 0x20, 0x8, 0x0, 0x0, 0x0, 0x0},
		{0x1, 0x1c, 0x20, 0x8, 0x0, 0x0, 0x0, 0x0, 0x81, 0x1c, 0x20, 0x8, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1c, 0x20, 0x28, 0x0, 0x0, 0x0, 0x0, 0x11, 0x1c, 0x20, 0x8, 0x0, 0x0},
		{0x0, 0x0, 0x3, 0x1c, 0x20, 0x8, 0x0, 0x0, 0x0, 0x0, 0x23, 0x1c, 0x20, 0x8, 0x0, 0x0, 0x0, 0x0, 0x43, 0x1c, 0x20, 0x8, 0x0, 0x0, 0x0, 0x0, 0x3, 0x1c, 0x20, 0x18},
		{0x0, 0x0, 0x0, 0x0, 0x3, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1c, 0x20, 0x1, 0x0, 0x0, 0x0, 0x0, 0x10, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0x1c},
		{0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x20, 0x2, 0x0, 0x0, 0x0, 0x0, 0x80, 0x4, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xa0, 0x0, 0x0, 0x0, 0x0, 0x0},
		{0x4, 0x14, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x14, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x14, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x10, 0x14, 0x20, 0x0, 0x0, 0x0},
		{0x0, 0x0, 0x2, 0x14, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x22, 0x14, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x42, 0x14, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x14, 0x20, 0x10}
	};

	address = 0xc3c0;
	for (int i = 0; i < 10; i++) {
		write_multi_i3c(address, &patchData2a[i][0], 30);
		address += 30;
	}
	
	const uint8_t patchData2b[20] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x1c, 0x20, 0x2, 0x0, 0x0, 0x0, 0x0, 0x22, 0x1c, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0};

	write_multi_i3c(address, &patchData2b[0], 20);

	const uint8_t patchData2c[25][30] = {
		{0x0, 0x0, 0x0, 0x0, 0x4, 0x10, 0x0, 0x0, 0x4, 0x20, 0x0, 0x0, 0x9d, 0x32, 0x0, 0x0, 0x4b, 0x40, 0x0, 0x0, 0x4, 0x50, 0x0, 0x0, 0x9d, 0x67, 0x0, 0x0, 0x2a, 0x71},
		{0x0, 0x0, 0x4, 0x80, 0x0, 0x0, 0xbe, 0x71, 0x0, 0x0, 0x4, 0x90, 0x0, 0x0, 0x25, 0x71, 0x0, 0x0, 0x4, 0xa0, 0x0, 0x0, 0xbe, 0x71, 0x0, 0x0, 0xdd, 0x65, 0x0, 0x0},
		{0x4, 0xb0, 0x0, 0x0, 0x54, 0x62, 0x0, 0x0, 0x4, 0x50, 0x0, 0x0, 0x6f, 0x66, 0x0, 0x0, 0x48, 0xc5, 0x0, 0x0, 0x2d, 0x61, 0x0, 0x0, 0x4, 0xd0, 0x0, 0x0, 0xdc, 0x60},
		{0x0, 0x0, 0xc2, 0xe1, 0x0, 0x0, 0x2c, 0xf1, 0x0, 0x0, 0x4, 0x0, 0x1, 0x0, 0xbe, 0xf1, 0x0, 0x0, 0x4, 0x10, 0x1, 0x0, 0xe9, 0xf2, 0x0, 0x0, 0x4, 0x20, 0x1, 0x0},
		{0x9d, 0xf7, 0x0, 0x0, 0x2a, 0x31, 0x1, 0x0, 0x4, 0x40, 0x1, 0x0, 0xbe, 0x31, 0x1, 0x0, 0x4, 0x50, 0x1, 0x0, 0x25, 0x31, 0x1, 0x0, 0x4, 0x60, 0x1, 0x0, 0x29, 0x31},
		{0x1, 0x0, 0x94, 0x70, 0x1, 0x0, 0xf0, 0xc2, 0x0, 0x0, 0xe, 0x82, 0x1, 0x0, 0xc2, 0xc1, 0x0, 0x0, 0xc3, 0x61, 0x0, 0x0, 0x4, 0x90, 0x1, 0x0, 0x6f, 0x66, 0x0, 0x0},
		{0x48, 0xc5, 0x0, 0x0, 0x2d, 0x61, 0x0, 0x0, 0x4, 0xa0, 0x1, 0x0, 0xdc, 0x60, 0x0, 0x0, 0x4, 0xb0, 0x1, 0x0, 0xbe, 0x31, 0x0, 0x0, 0x4, 0xc0, 0x1, 0x0, 0xa, 0x30},
		{0x0, 0x0, 0x4, 0xd0, 0x1, 0x0, 0xa, 0x10, 0x0, 0x0, 0x0, 0x0, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x10, 0x0, 0x0, 0x4, 0x20, 0x0, 0x0, 0xe9, 0x32},
		{0x0, 0x0, 0x4, 0xe0, 0x1, 0x0, 0xc8, 0xf8, 0x1, 0x0, 0x4, 0x0, 0x2, 0x0, 0x8a, 0xfa, 0x1, 0x0, 0x4, 0x10, 0x2, 0x0, 0x9d, 0xf7, 0x1, 0x0, 0x2a, 0x21, 0x2, 0x0},
		{0x4, 0x30, 0x2, 0x0, 0xbe, 0x21, 0x2, 0x0, 0x4, 0x40, 0x2, 0x0, 0x25, 0x21, 0x2, 0x0, 0x4, 0x50, 0x2, 0x0, 0xbe, 0x21, 0x2, 0x0, 0xfe, 0xff, 0x1, 0x0, 0xfe, 0xff},
		{0x1, 0x0, 0xfe, 0xff, 0x1, 0x0, 0xfe, 0xff, 0x1, 0x0, 0xec, 0xf1, 0x1, 0x0, 0x4b, 0x60, 0x0, 0x0, 0x4, 0x50, 0x0, 0x0, 0x6f, 0x66, 0x0, 0x0, 0x48, 0xc5, 0x0, 0x0},
		{0x2d, 0x61, 0x0, 0x0, 0x4, 0xd0, 0x0, 0x0, 0xdc, 0x60, 0x0, 0x0, 0xe, 0x82, 0x1, 0x0, 0xc2, 0xc1, 0x0, 0x0, 0xc3, 0x61, 0x0, 0x0, 0x4, 0x90, 0x1, 0x0, 0x6f, 0x66},
		{0x0, 0x0, 0x48, 0xc5, 0x0, 0x0, 0x2d, 0x61, 0x0, 0x0, 0x4, 0xa0, 0x1, 0x0, 0xdc, 0x60, 0x0, 0x0, 0x4, 0x60, 0x2, 0x0, 0xbe, 0x61, 0x0, 0x0, 0x4, 0xc0, 0x1, 0x0},
		{0xa, 0x30, 0x0, 0x0, 0x4, 0xd0, 0x1, 0x0, 0x91, 0x10, 0x0, 0x0, 0x0, 0x0, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x10, 0x0, 0x0, 0x4, 0x20, 0x0, 0x0, 0x9d, 0x32, 0x0, 0x0},
		{0x4b, 0x40, 0x0, 0x0, 0x4, 0x90, 0x1, 0x0, 0x9d, 0x67, 0x0, 0x0, 0x2a, 0x71, 0x0, 0x0, 0x4, 0x70, 0x2, 0x0, 0xbe, 0x71, 0x0, 0x0, 0x4, 0x90, 0x0, 0x0, 0x25, 0x71},
		{0x0, 0x0, 0x4, 0xa0, 0x0, 0x0, 0xbe, 0x71, 0x0, 0x0, 0x0, 0x60, 0x0, 0x0, 0x4, 0xb0, 0x0, 0x0, 0x54, 0x62, 0x0, 0x0, 0xe, 0x82, 0x1, 0x0, 0xc2, 0xc1, 0x0, 0x0},
		{0xc3, 0x61, 0x0, 0x0, 0x4, 0x90, 0x1, 0x0, 0x6f, 0x66, 0x0, 0x0, 0x48, 0xc5, 0x0, 0x0, 0x2d, 0x61, 0x0, 0x0, 0x4, 0xa0, 0x1, 0x0, 0x9, 0x62, 0x0, 0x0, 0x4, 0xb0},
		{0x1, 0x0, 0xbe, 0x31, 0x0, 0x0, 0x4, 0xc0, 0x1, 0x0, 0xa, 0x30, 0x0, 0x0, 0xf, 0x10, 0x0, 0x0, 0x0, 0x0, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x10},
		{0x0, 0x0, 0x4, 0x20, 0x0, 0x0, 0x9d, 0x32, 0x0, 0x0, 0x4b, 0x40, 0x0, 0x0, 0x4, 0x50, 0x0, 0x0, 0x9d, 0x67, 0x0, 0x0, 0x2a, 0x71, 0x0, 0x0, 0x4, 0x80, 0x0, 0x0},
		{0xbe, 0x71, 0x0, 0x0, 0x4, 0x90, 0x0, 0x0, 0x25, 0x71, 0x0, 0x0, 0x4, 0xa0, 0x0, 0x0, 0xbe, 0x71, 0x0, 0x0, 0x0, 0x60, 0x0, 0x0, 0x4, 0xb0, 0x0, 0x0, 0x54, 0x62},
		{0x0, 0x0, 0xe, 0x82, 0x1, 0x0, 0xc2, 0xc1, 0x0, 0x0, 0xc3, 0x61, 0x0, 0x0, 0x4, 0x50, 0x0, 0x0, 0x6f, 0x66, 0x0, 0x0, 0x48, 0xc5, 0x0, 0x0, 0x2d, 0x61, 0x0, 0x0},
		{0x4, 0xd0, 0x0, 0x0, 0x9, 0x62, 0x0, 0x0, 0x4, 0xb0, 0x1, 0x0, 0xbe, 0x31, 0x0, 0x0, 0x4, 0xc0, 0x1, 0x0, 0xa, 0x30, 0x0, 0x0, 0x4, 0xd0, 0x1, 0x0, 0xa, 0x10}
	};

	address = 0xc640;
	for (int i = 0; i < 25; i++) {
		write_multi_i3c(address, &patchData2c[i][0], 30);
		address += 30;
	}

	const uint8_t patchData2d[6] = {0x0, 0x0, 0x0, 0x0, 0x60, 0x0};

	write_multi_i3c(address, &patchData2d[0], 6);

	// Run command END_VTRAM_UPDATE
	write_multi_i3c_1byte(VD55G1_CMD_STBY, 0x7);
	HAL_Delay(200);
	ReturnValue = MultiPollingValueWithMask(4, VD55G1_CMD_STBY, 0x0, 0xf, 1); // (val=0) UI.CMD.STBY.COMMAND //not working on 10/11
	printf("Run command END_VTRAM_UPDATE: %d\n",ReturnValue);

	// =====================
	// Operations in SW_STBY
	// =====================
	// Set frequencies
	write_multi_i3c_4bytes(UI_SENSOR_SETTINGS_EXT_CLOCK_VALUE, 0x0, 0x1b, 0xb7, 0x0); // (val=12000000) / UI.SENSOR_SETTINGS.EXT_CLOCK.VALUE
	write_multi_i3c_4bytes(UI_SENSOR_SETTINGS_MIPI_DATA_RATE_VALUE, 0x0, 0x8c, 0x86, 0x47); // (val=1200000000) / UI.SENSOR_SETTINGS.MIPI_DATA_RATE.VALUE
	write_multi_i3c_2bytes(UI_STREAM_STATICS_LINE_LENGTH_VALUE, 0x68, 0x4); // (val=1128) / UI.STREAM_STATICS.LINE_LENGTH.VALUE
	// => External clock = 12.00 MHz, Csi frequency = 1200.00 Mbps, Line time = 7.52 us

	write_multi_i3c_1byte(UI_STREAM_CTX0_EXPOSURE_MODE_MODE, 0x0);

	write_multi_i3c_2bytes(UI_STREAM_STATICS_LINE_LENGTH_VALUE,  0xc8, 0xaf);
	write_multi_i3c_2bytes(UI_STREAM_CTX0_FRAME_LENGTH_VALUE, 0x82, 0x06);
	write_multi_i3c_4bytes(UI_STREAM_CTX1_FRAME_LENGTH_VALUE, 0x82, 0x06, 0x0, 0x0); 
	write_multi_i3c_4bytes(UI_STREAM_CTX2_FRAME_LENGTH_VALUE, 0x82, 0x06, 0x0, 0x0); 
	write_multi_i3c_4bytes(UI_STREAM_CTX3_FRAME_LENGTH_VALUE, 0x82, 0x06, 0x0, 0x0);

	write_multi_i3c_1byte(UI_STREAM_CTX0_READOUT_CTRL, 0x1); // ctx0 READOUT_CTRL, 2x binning

	write_multi_i3c_2bytes(UI_STREAM_CTX0_X_WIDTH, 0x20, 0x3); // (val=800) / UI.STREAM_CTX0.X_WIDTH
	write_multi_i3c_2bytes(UI_STREAM_CTX0_X_START, 0x0, 0x0); // (val=0) / UI.STREAM_CTX0.X_START
	write_multi_i3c_2bytes(UI_STREAM_CTX0_Y_HEIGHT, 0xb8, 0x2); // (val=696) / UI.STREAM_CTX0.Y_HEIGHT
	write_multi_i3c_2bytes(UI_STREAM_CTX0_Y_START, 0x0, 0x0); // (val=0) / UI.STREAM_CTX0.Y_START

	write_multi_i3c_1byte(UI_STREAM_CTX0_GPIO_0_CTRL, 0x0c); // (val=12) / UI.STREAM_CTX0.GPIO_0_CTRL
	// Exposure
	write_multi_i3c_1byte(UI_STREAM_CTX0_EXPOSURE_MODE_MODE, 0x0); //  UI.STREAM_CTX0.EXPOSURE_MODE.MODE = 0
}


int ls55g1_init(void)
{
	// check the cut version
	uint8_t DeviceID;
	read_multi_i3c(UI_STATUS_DEVICE_DEVICE_ID, &DeviceID, 1);
	// Initialize the VD55G1
	if (DeviceID == 0x20)
	{
		ls55g1_init_cut20();
	} else {
		return -1; // cut not supported
	}
	return 0;
}

int ls55g1_set_digital_gain(float DigitalGain)
{
	float _DigitalGain = DigitalGain;
	uint8_t DigitalGainMSB, DigitalGainLSB;

	if (_DigitalGain < 1.0)
	{
		_DigitalGain = 1.0;
	}
	if (_DigitalGain > 32.0)
	{
		_DigitalGain = 32.0;
	}
	DigitalGainMSB = (int)round(_DigitalGain);
	DigitalGainLSB = (int)((_DigitalGain - DigitalGainMSB)*256);
	write_multi_i3c_2bytes(EXPOSURE_MANUAL_DIGITAL_GAIN_CH0, DigitalGainLSB, DigitalGainMSB);

	Config.DigitalGain = DigitalGainMSB + DigitalGainLSB/256.0;

	return 0;
}

int ls55g1_set_framelength(int FrameLength)
{
	int _FrameLength = FrameLength;
	if (_FrameLength < 700)
	{
		_FrameLength = 700;
	}
	write_multi_i3c_4bytes( UI_STREAM_CTX0_FRAME_LENGTH_VALUE, _FrameLength & 0xFF, (_FrameLength & 0xFF00) / 0x100, 0x00, 0x00);

	Config.FrameLength = _FrameLength;

	return 0;

}

int ls55g1_set_exposure(int Exposure)
{
	int _Exposure = Exposure;
	if (_Exposure > Config.FrameLength)
	{
		_Exposure = Config.FrameLength;
	}
	write_multi_i3c_2bytes( EXPOSURE_MANUAL_COARSE_EXPOSURE_LINES_A, _Exposure & 0xFF, (_Exposure & 0xFF00) / 0x100 );

	Config.Exposure =  _Exposure;

	return 0;

}

int ls55g1_set_autogain(int AutoGain)
{
	if (AutoGain == 1)
	{
		write_multi_i3c_1byte( UI_STREAM_CTX0_EXPOSURE_MODE_MODE, 0x00); 
	}
	else
	{
		write_multi_i3c_1byte( UI_STREAM_CTX0_EXPOSURE_MODE_MODE, 0x02);
	}

	Config.AutoGain =  AutoGain;

	return 0;

}

int ls55g1_set_analoggain(int AnalogGain)
{
	if (Config.AutoGain == 0)
	{
		write_multi_i3c_1byte( EXPOSURE_MANUAL_ANALOG_GAIN, AnalogGain);
		Config.AnalogGain =  AnalogGain;
	}

	return 0;

}

int ls55g1_get_camera_status(int *pStatus)
{
	uint8_t SensorStatus;
	read_multi_i3c(VD55G1_SYSTEM_FSM, &SensorStatus, 1);

	*pStatus = (int)SensorStatus;

	return 0;

}

int ls55g1_start_streaming(void)
{
	write_multi_i3c_1byte(VD55G1_CMD_STBY, 0x1);

	return 0;

}

int ls55g1_stop_streaming(int *pStatus)
{
	uint8_t SensorStatus;
	read_multi_i3c(VD55G1_SYSTEM_FSM, &SensorStatus, 1);

	*pStatus = (int)SensorStatus;

	return 0;

}

int ls55g1_set_binning2x2(int binning2x2)
{
	if ((Config.Sensor_X_Start % 2) == 1)
	{
		Config.Sensor_X_Start--;
	}
	if ((Config.Sensor_Y_Start % 2) == 1)
	{
		Config.Sensor_Y_Start--;
	}
	if (Config.Binning2x2 > 1)
	{
		Config.Binning2x2 = 1;
	}
	if (Config.Binning2x2  == 0)
	{
		write_multi_i3c_2bytes( UI_STREAM_CTX0_Y_START, Config.Sensor_Y_Start & 0xFF, (Config.Sensor_Y_Start & 0xFF00) / 0x100); 	
		write_multi_i3c_2bytes( UI_STREAM_CTX0_Y_HEIGHT, 0x90, 0x01); 																
		write_multi_i3c_2bytes( UI_STREAM_CTX0_X_START, Config.Sensor_X_Start & 0xFF, (Config.Sensor_X_Start & 0xFF00) / 0x100); 	
		write_multi_i3c_2bytes( UI_STREAM_CTX0_X_WIDTH, 0x90, 0x01); 																
		write_multi_i3c_1byte( UI_STREAM_CTX0_READOUT_CTRL, 0);
	}
	else
	{
		write_multi_i3c_2bytes( UI_STREAM_CTX0_Y_START, 0x00, 0x00); 	
		write_multi_i3c_2bytes( UI_STREAM_CTX0_Y_HEIGHT, 0xB8, 0x02); 	
		write_multi_i3c_2bytes( UI_STREAM_CTX0_X_START, 0x00, 0x00); 	
		write_multi_i3c_2bytes( UI_STREAM_CTX0_X_WIDTH, 0x20, 0x03); 	
		write_multi_i3c_1byte( UI_STREAM_CTX0_READOUT_CTRL, 1); 
	}

	return 0;
}


void ls55g1_configure(uint16_t Width, uint16_t Height, uint8_t Binning_Subsample)
{
	/*	0x00: = normal streaming
		0x01: = digital binning x2
		0x02: = digital binning x4
		0x03: = subsampling x2
		0x04: = subsampling x4
		0x05: = subsampling x8
		0x06: = XY binning x2
		*/
	uint8_t WidthLSB, WidthMSB, HeightLSB, HeightMSB, CoarseExpMSB, CoarseExpLSB, FrameLenMSB, FrameLenLSB;
	uint8_t  Registers[20];

	MultiPollingValueWithMask(200, 0x001c, 0x02, 0xFF, 10); //POLL_Register STATUS_SYSTEM_FSM for sensor to be in Standby

	if((Binning_Subsample == 1) || (Binning_Subsample == 3))
	{
		Width=Width*2;
		Height=Height*2;
	}
	WidthLSB = Width & 0xFF;
	WidthMSB = (Width & 0xFF00) / 0x100;

	HeightLSB = Height & 0xFF;
	HeightMSB = (Height & 0xFF00) / 0x100;



	FrameLenLSB = Config.FrameLength & 0xFF;
	FrameLenMSB = (Config.FrameLength & 0xFF00) / 0x100;
	write_multi_i3c_4bytes( UI_STREAM_CTX0_FRAME_LENGTH_VALUE, FrameLenLSB, FrameLenMSB, 0x00, 0x00);

	if(Binning_Subsample == 0)
	{
		write_multi_i3c_2bytes( UI_STREAM_STATICS_LINE_LENGTH_VALUE, 0x38, 0xC7);
	}
	else
	{
		write_multi_i3c_2bytes( UI_STREAM_STATICS_LINE_LENGTH_VALUE, 0x00, 0x6f);
	}


	write_multi_i3c_2bytes( STREAM_STATICS_FORMAT_CTRL, 0x08, 0x00); 			
	write_multi_i3c_2bytes( STREAM_STATICS_I3C_FRAME_READOUT_CTRL, 0x03, 0x00);
	write_multi_i3c_1byte( STREAM_STATICS_ISL_ENABLE, 0x00);  	
	write_multi_i3c_1byte( STREAM_STATICS_DARKCAL_CTRL, 0x42); 						
	write_multi_i3c_2bytes( STREAM_STATICS_CONTEXT_NEXT_CONTEXT, 0x11, 0x11);	

	read_multi_i3c(EXPOSURE_STEP_PROPORTION_A, Registers, 2);
	printf("x0488 = %x, %x\n", Registers[0] ,Registers[1]);
	write_multi_i3c_2bytes( EXPOSURE_STEP_PROPORTION_A, 0xF0, 0x00);	

	// *** Context 0 Config ***
	if (Config.AutoGain == 1)
	{
		write_multi_i3c_1byte( UI_STREAM_CTX0_EXPOSURE_MODE_MODE, 0x0); 
	}
	else
	{
		write_multi_i3c_1byte( UI_STREAM_CTX0_EXPOSURE_MODE_MODE, 0x2);
		write_multi_i3c_1byte( EXPOSURE_MANUAL_ANALOG_GAIN, Config.AnalogGain); 
		if (Config.Exposure > Config.FrameLength)
		{
			Config.Exposure = Config.FrameLength;
		}
		CoarseExpLSB = Config.Exposure & 0xFF;
		CoarseExpMSB = (Config.Exposure & 0xFF00) / 0x100;

		write_multi_i3c_2bytes( EXPOSURE_MANUAL_COARSE_EXPOSURE_LINES_A, CoarseExpLSB, CoarseExpMSB); 
	}

	if (Config.Sensor_X_Start> 400)
	{
		Config.Sensor_X_Start = 400;
	}
	if (Config.Sensor_Y_Start> 350)
	{
		Config.Sensor_Y_Start = 350;
	}
	if (Binning_Subsample == 0)
	{
		write_multi_i3c_2bytes( UI_STREAM_CTX0_Y_START, Config.Sensor_Y_Start & 0xFF, (Config.Sensor_Y_Start & 0xFF00) / 0x100); 
		write_multi_i3c_2bytes( UI_STREAM_CTX0_X_START, Config.Sensor_X_Start & 0xFF, (Config.Sensor_X_Start & 0xFF00) / 0x100);

	}
	else
	{
		write_multi_i3c_2bytes( UI_STREAM_CTX0_Y_START, 0x00, 0x00);
		write_multi_i3c_2bytes( UI_STREAM_CTX0_X_START, 0x00, 0x00); 
	}

	write_multi_i3c_2bytes( UI_STREAM_CTX0_Y_HEIGHT, HeightLSB, HeightMSB); 

	write_multi_i3c_2bytes( UI_STREAM_CTX0_X_WIDTH, WidthLSB, WidthMSB); 
	write_multi_i3c_1byte( UI_STREAM_CTX0_GPIO_0_CTRL, 0x0C);		
	write_multi_i3c_1byte( UI_STREAM_CTX0_GPIO_1_CTRL, 0x00);		
	write_multi_i3c_1byte( UI_STREAM_CTX0_GPIO_2_CTRL, 0x00);		
	write_multi_i3c_1byte( UI_STREAM_CTX0_GPIO_3_CTRL, 0x00);		
	write_multi_i3c_1byte( UI_STREAM_CTX0_READOUT_CTRL, Binning_Subsample); 
	write_multi_i3c_1byte( UI_STREAM_CTX0_MASK_FRAME_CTRL, 0x0);	

	write_multi_i3c_2bytes( CONTEXT_NEXT_CONTEXT, 0x11, 0x11);  		
}
