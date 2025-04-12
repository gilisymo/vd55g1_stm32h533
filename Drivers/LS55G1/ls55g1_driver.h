/**
  ******************************************************************************
  * @file          : ls55g1_driver.h
  * @author        : Giovanni@Gilisymo
  * @brief         : This file provides header code to drive the VD55G1.
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
#ifndef INC_LS55G1_DRIVER_H_
#define INC_LS55G1_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif


#define LS55G1_VERSION_MAJOR 1
#define LS55G1_VERSION_MINOR 0
#define LS55G1_VERSION_PATCH 0

typedef struct {
	int AutoGain;
	int Binning2x2;
	int Exposure;
	float AnalogGain;
	int FrameLength;
	int Sensor_X_Start;
	int Sensor_Y_Start;
	float DigitalGain;
} ls55g1_config_t;

typedef struct {
	uint8_t major; /* API changes not compatibles */
	uint8_t minor; /* Campatibles changes */
	uint8_t patch; /* Bug fixes */
} ls55g1_driver_version_t;

// Register from UM3224
#define VD55G1_SYSTEM_FSM 	0x001C
#define VD55G1_CMD_STBY 	  0x0201
#define UI_STATUS_DEVICE_MODEL_ID  0x0000
#define UI_STATUS_DEVICE_DEVICE_ID  0x0004
#define START_PATCH_ADDRESS 0x2000
#define UI_CMD_BOOT_COMMAND 0x0200
#define UI_SENSOR_SETTINGS_EXT_CLOCK_VALUE 0x0220
#define UI_SENSOR_SETTINGS_MIPI_DATA_RATE_VALUE 0x0224
#define UI_STREAM_STATICS_LINE_LENGTH_VALUE 0x0300
#define UI_STREAM_CTX0_EXPOSURE_MODE_MODE 0x0500
#define UI_STREAM_CTX0_FRAME_LENGTH_VALUE 0x050c
#define UI_STREAM_CTX1_FRAME_LENGTH_VALUE 0x055c
#define UI_STREAM_CTX2_FRAME_LENGTH_VALUE 0x05ac
#define UI_STREAM_CTX3_FRAME_LENGTH_VALUE 0x05af
#define UI_STREAM_CTX0_READOUT_CTRL 0x052e
#define UI_STREAM_CTX0_Y_START 0x0510
#define UI_STREAM_CTX0_Y_HEIGHT 0x0512
#define UI_STREAM_CTX0_X_START 0x0514
#define UI_STREAM_CTX0_X_WIDTH 0x0516
#define EXPOSURE_MANUAL_DIGITAL_GAIN_CH0 0x0504
#define EXPOSURE_MANUAL_COARSE_EXPOSURE_LINES_A 0x0502
#define EXPOSURE_MANUAL_ANALOG_GAIN 0x0501
#define UI_STREAM_CTX0_GPIO_0_CTRL 0x051d
#define UI_STREAM_CTX0_GPIO_1_CTRL 0x051e
#define UI_STREAM_CTX0_GPIO_2_CTRL 0x051f
#define UI_STREAM_CTX0_GPIO_3_CTRL 0x0520
#define UI_STREAM_CTX0_MASK_FRAME_CTRL 0x0537
#define CONTEXT_NEXT_CONTEXT 0x03e4
#define STREAM_STATICS_FORMAT_CTRL 0x030a
#define STREAM_STATICS_I3C_FRAME_READOUT_CTRL 0x0324
#define STREAM_STATICS_ISL_ENABLE 0x0326
#define STREAM_STATICS_DARKCAL_CTRL 0x032a
#define STREAM_STATICS_CONTEXT_NEXT_CONTEXT 0x0410
#define EXPOSURE_STEP_PROPORTION_A 0x0488
#define EXPOSURE_MANUAL_ANALOG_GAIN 0x0501
#define EXPOSURE_MANUAL_COARSE_EXPOSURE_LINES_A 0x0502

void ls55g1_get_driver_version(ls55g1_driver_version_t *p_driver_version);
int ls55g1_init(void);
int ls55g1_set_digital_gain(float DigitalGain);
int ls55g1_set_framelength(int FrameLength);
int ls55g1_set_exposure(int Exposure);
int ls55g1_set_autogain(int AutoGain);
int ls55g1_set_analoggain(int AnalogGain);
void ls55g1_configure(uint16_t Width, uint16_t Height, uint8_t Binning_Subsample);
int ls55g1_get_camera_status(int *pStatus);
int ls55g1_start_streaming(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_LS55G1_DRIVER_H_ */
