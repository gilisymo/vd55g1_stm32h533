/**
  ******************************************************************************
  * @file          : ls55g1_app.c
  * @author        : Giovanni@Gilisymo
  * @brief         : This file provides code for the demo application.
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

#include "ls55g1_app.h"

static void ls_cam_i3c_reset(void);
static void send_to_display(void);
static void print_camera_status(void);

#define MONO2COLOR565(mono) { ((mono & 0xF8) << 8) | ((mono & 0xFC) << 3) | ((mono & 0xF8) >> 3) }
#define COLOR565(red, green, blue) { ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3) }

uint8_t ImageBuffer[400][400];
uint8_t DisplayBuffer[800];

#define CameraHeight	240
#define CameraWidth		320

extern ls55g1_config_t config;
volatile uint8_t LineDataReady = 0;
volatile uint8_t SPIDMATxComplete = 0;
volatile uint8_t DMALineDataReady = 0;
volatile uint8_t MultipleXferCplt = 0;

static uint16_t linecounter = 0;
static uint32_t FrameCount = 0;
static ls55g1_driver_version_t driver_version = {0, 0, 0};

extern I3C_HandleTypeDef hi3c1;


void ls55g1_setup(void)
{
	// Initialize the Display
	ILI9341_Init();
	ILI9341_Fill_Screen(BLUE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("GILISYMO", 20, 20, WHITE, 2, BLUE);
	HAL_Delay(1000);
	ILI9341_Draw_Text("LS-CAM-I3C", 20, 50, WHITE, 2, BLUE);
	HAL_Delay(1000);
	ILI9341_Draw_Text("V1.0", 20, 80, WHITE, 2, BLUE);
	HAL_Delay(1000);
	ILI9341_Fill_Screen(BLUE);
	ILI9341_Set_Rotation(SCREEN_VERTICAL_1);

}


void ls55g1_main(void)
{
	uint8_t DeviceID[4];

	printf("GILISYMO LS-I3C-CAM VD55G1\n");

	// Print the version of the demo application
	printf("Demo Version: %d.%d.%d\n", DEMO_VERSION_MAJOR, DEMO_VERSION_MINOR, DEMO_VERSION_PATCH);

	// Print the version of the driver
	ls55g1_get_driver_version(&driver_version);
	printf("LS55G1 Driver Version: %d.%d.%d\n", driver_version.major, driver_version.minor, driver_version.patch);

	printf("Camera Booting...\n");
	setvbuf(stdin, NULL, _IONBF, 0);

	// Reset the Camera
	ls_cam_i3c_reset();

	I3C_Config();

	// Clear DeviceID array
	memset(DeviceID, 0, sizeof(DeviceID));
	// Clear ImageBuffer array
	memset(ImageBuffer, 0, sizeof(ImageBuffer));

	read_multi_i3c(0x0000, DeviceID, 12);
	Printf_Warn("CAMERA Device ID = %c%c%c%c\n",
			DeviceID[3] ,DeviceID[2],DeviceID[1],DeviceID[0]);

	// Initialize the Camera
	if (ls55g1_init()) {
		Printf_Error("VD55G1 cut not supported!\n");
		Error_Handler();
	}

	ls55g1_configure(CameraWidth, CameraHeight, config.Binning2x2);

	// Set Parameters
	ls55g1_set_digital_gain(1.1);
	ls55g1_set_framelength(700);
	ls55g1_set_exposure(500);

	// Start Camera Streaming
	Printf_Warn("\nCamera Start!!\n");

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO0_Pin);
	HAL_NVIC_ClearPendingIRQ(EXTI7_IRQn);
	HAL_NVIC_EnableIRQ(EXTI7_IRQn);
	LineDataReady = 0;
	linecounter=0;

	ls55g1_start_streaming();
	Printf_Warn("START Streaming\n");

	print_camera_status();

	// If frequency is too low the sensor will go to error. If frequency is too high
	// there will be interferences and the sensor will go to error.
	SetI3CFreq(9500000);


	while (1)
	{

		// LineDataReady is set when an interrupt from the LS CAM arrive in the GPIO0
		// We will have 1 interrupt for each line of the image
		if (LineDataReady == 1)
		{
			// Read one line from the sensor
			//read_multi_i3c(0x2000, ImageBuffer[linecounter], CameraWidth);
            read_multi_i3c_async(&hi3c1, 0x2000, ImageBuffer[linecounter], CameraWidth);

      		// wait end of DMA
			while (!DMALineDataReady)
			{
			}
			LineDataReady = 0;
			DMALineDataReady = 0;


			// Interrupts are disabled in the interrupt callback to avoid to lose the next line
			// The following code will reenable the interrupt on GPIO0 since the line was previously read
			__HAL_GPIO_EXTI_CLEAR_IT(GPIO0_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI7_IRQn);
			HAL_NVIC_EnableIRQ(EXTI7_IRQn);

			linecounter++;
		}

		// The following code will be run when the image captured is completed
		if (linecounter >= CameraHeight)
		{

			FrameCount++;
			linecounter=0;
			Printf_Warn("FrameCount=%d\n",(int)FrameCount);

			print_camera_status();

			// Send data to the display
			send_to_display();

		}


	}


}


void HAL_I3C_CtrlMultipleXferCpltCallback(I3C_HandleTypeDef *hi3c)
{
    // Transfer complete, update state
	MultipleXferCplt = 1;
}

void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c) {
    DMALineDataReady = 1;
}

void HAL_I3C_ErrorCallback(I3C_HandleTypeDef *hi3c)
{
	// TODO: Add error handling code here
    // Handle error, update state
    hi3c->State = HAL_I3C_STATE_READY;
    // Additional error handling code
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) /* Derogation MISRAC2012-Rule-8.13 */
{
 /* Prevent unused argument(s) compilation warning */
	SPIDMATxComplete = 1;
 /* NOTE : This function should not be modified, when the callback is needed,
           the HAL_SPI_TxCpltCallback should be implemented in the user file
  */
}

static void ls_cam_i3c_reset(void)
{
	  //Hardware enabling
	HAL_GPIO_WritePin(NRST_GPIO_Port, NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(NRST_GPIO_Port, NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

}
static void print_camera_status(void)
{
	int Status;
	ls55g1_get_camera_status(&Status);
	Printf_Warn("Sensor Status=%d\n", Status);
}


static void send_to_display(void)
{
	ILI9341_Set_Address(0, 0, CameraHeight, CameraWidth );
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);	//CS ON
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);	// DATA
	for (int i = 0; i < CameraWidth; i++)
	{
		 for (int j = 0; j < CameraHeight; j++) {
			 uint8_t data = ImageBuffer[j][i];
			 uint16_t color = MONO2COLOR565(data);
			 DisplayBuffer[2*j] = (uint8_t)(color>>8);
			 DisplayBuffer[2*j+1] = (uint8_t)(color & 0xFF);
		 }

		 SPIDMATxComplete = 0;
		 HAL_SPI_Transmit(HSPI_INSTANCE, (unsigned char *)DisplayBuffer, 2*CameraHeight, 10);
		 //HAL_SPI_Transmit_DMA(HSPI_INSTANCE, (unsigned char *)DisplayBuffer, 2*CameraHeight);
		 //while(!SPIDMATxComplete)
		 {
		 }
	}
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);	//CS OFF
}

