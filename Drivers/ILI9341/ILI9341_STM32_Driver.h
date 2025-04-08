
//	MIT License
//
//	Copyright (c) 2017 Matej Artnak
//
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//
//
//
//-----------------------------------
//	ILI9341 Driver library for STM32
//-----------------------------------
//
//	While there are other libraries for ILI9341 they mostly require either interrupts, DMA or both for fast drawing
//	The intent of this library is to offer a simple yet still reasonably fast alternatives for those that
//	do not wish to use interrupts or DMA in their projects.
//
//	Library is written for STM32 HAL library and supports STM32CUBEMX. To use the library with Cube software
//	you need to tick the box that generates peripheral initialization code in their own respective .c and .h file
//
//
//-----------------------------------
//	Performance
//-----------------------------------
//	Settings:	
//	--SPI @ 50MHz 
//	--STM32F746ZG Nucleo board
//	--Redraw entire screen
//
//	++		Theoretical maximum FPS with 50Mhz SPI calculated to be 40.69 FPS
//	++		320*240 = 76800 pixels, each pixel contains 16bit colour information (2x8)
//	++		Theoretical Max FPS: 1/((320*240*16)/50000000)
//
//	With ART Accelerator, instruction prefetch, CPI ICACHE and CPU DCACHE enabled:
//
//	-FPS:									39.62
//	-SPI utilization:			97.37%
//	-MB/Second:						6.09
//
//	With ART Accelerator, instruction prefetch, CPI ICACHE and CPU DCACHE disabled:
//
//	-FPS:									35.45
//	-SPI utilization:			87.12%
//	-MB/Second:						5.44
//	
//	ART Accelerator, instruction prefetch, CPI ICACHE and CPU DCACHE settings found in MXCUBE under "System-> CORTEX M7 button"
//
//
//
//-----------------------------------
//	How to use this library
//-----------------------------------
//
//	-generate SPI peripheral and 3 GPIO_SPEED_FREQ_VERY_HIGH GPIO outputs
//	 		++Library reinitializes GPIOs and SPIs generated by gpio.c/.h and spi.c/.h using MX_X_Init(); calls
//			++reinitialization will not clash with previous initialization so generated initializations can be laft as they are
//	-If using MCUs other than STM32F7 you will have to change the #include "stm32f7xx_hal.h" in the ILI9341_STM32_Driver.h to your respective .h file
//	-define your HSPI_INSTANCE in ILI9341_STM32_Driver.h
//	-define your CS, DC and RST outputs in ILI9341_STM32_Driver.h
//	-check if ILI9341_SCREEN_HEIGHT and ILI9341_SCREEN_WIDTH match your LCD size
//			++Library was written and tested for 320x240 screen size. Other sizes might have issues**
//	-in your main program initialize LCD with ILI9341_Init();
//	-library is now ready to be used. Driver library has only basic functions, for more advanced functions see ILI9341_GFX library	
//
//-----------------------------------


#ifndef ILI9341_STM32_DRIVER_H
#define ILI9341_STM32_DRIVER_H

#include "stm32h5xx_hal.h"
#include "main.h"

#define ILI9341_SCREEN_HEIGHT 240 
#define ILI9341_SCREEN_WIDTH 	320

extern SPI_HandleTypeDef hspi2;

//SPI INSTANCE
#define HSPI_INSTANCE							&hspi2

//CHIP SELECT PIN AND PORT, STANDARD GPIO
#define LCD_CS_PORT								LCD_CS_GPIO_Port
#define LCD_CS_PIN								LCD_CS_Pin

//DATA COMMAND PIN AND PORT, STANDARD GPIO
#define LCD_DC_PORT								LCD_DC_GPIO_Port
#define LCD_DC_PIN								LCD_DC_Pin

//RESET PIN AND PORT, STANDARD GPIO
#define	LCD_RST_PORT							LCD_RST_GPIO_Port
#define	LCD_RST_PIN								LCD_RST_Pin


#define BURST_MAX_SIZE 	500

#define BLACK       0x0000      
#define NAVY        0x000F      
#define DARKGREEN   0x03E0      
#define DARKCYAN    0x03EF      
#define MAROON      0x7800      
#define PURPLE      0x780F      
#define OLIVE       0x7BE0      
#define LIGHTGREY   0xC618      
#define DARKGREY    0x7BEF      
#define BLUE        0x001F      
#define GREEN       0x07E0      
#define CYAN        0x07FF      
#define RED         0xF800     
#define MAGENTA     0xF81F      
#define YELLOW      0xFFE0      
#define WHITE       0xFFFF      
#define ORANGE      0xFD20      
#define GREENYELLOW 0xAFE5     
#define PINK        0xF81F

#define SCREEN_VERTICAL_1			0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2			2
#define SCREEN_HORIZONTAL_2		3

void ILI9341_SPI_Init(void);
void ILI9341_SPI_Send(unsigned char SPI_Data);
void ILI9341_Write_Command(uint8_t Command);
void ILI9341_Write_Data(uint8_t Data);
void ILI9341_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void ILI9341_Reset(void);
void ILI9341_Set_Rotation(uint8_t Rotation);
void ILI9341_Enable(void);
void ILI9341_Init(void);
void ILI9341_Fill_Screen(uint16_t Colour);
void ILI9341_Draw_Colour(uint16_t Colour);
void ILI9341_Draw_Pixel(uint16_t X,uint16_t Y,uint16_t Colour);
void ILI9341_Draw_Colour_Burst(uint16_t Colour, uint32_t Size);


void ILI9341_Draw_Rectangle(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t Colour);
void ILI9341_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour);
void ILI9341_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Colour);
	
#endif

