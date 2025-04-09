# GILISYMO LS-CAM-I3C STM32 TFT DEMO

## LS-CAM-I3C Pinout Connection

## Pinout Description

| Pin | Description                       |
|-----|-----------------------------------|
| 1   | Vin: Input voltage +3.3V or +5V   |
| 2   | 1v8: Output from LDO for Promodule |
| 3   | GND: Ground                       |
| 4   | SCL: I3C Clock                    |
| 5   | SDA: I3C Data                     |
| 6   | GPIO0: GPIO used for LINE interrupt|
| 7   | GPIO1: GPIO                       |
| 8   | NRST: Reset                       |

> **Warning:** In the pre-release PCB version, the pinout silkscreen is not correct.

## Demo Example with NUCLEO-H533RE

This demo example works with the NUCLEO-H533RE. The Light Saber CAM can be connected as follows:

| CN10 Pin | LS CAM Pin |
|----------|------------|
| 3  (PB6) | 4 (SCL)    |
| 5  (PB7) | 5 (SDA)    |
| 7  (AVDD)| 1 (Vin)    |
| 9  (GND) | 3 (GND)    |
| 13 (PA6) | 7 (GPIO1)  |
| 15 (PA7) | 6 (GPIO0)  |
| 17 (PC9) | 8 (NRST)   |

> **Note:** In the `um3121-stm32h5-nucleo64-board-mb1814-stmicroelectronics.pdf` (page 28), you can find the description of the pin assignment of the ST morpho connector.

> **Warning:** Keep the connections short, or the demo will hang. Alternatively, you can slow down the I3C clock frequency. However, in that case, reading from the device will take more time, and the display will not have time to update.

## Display Information

The display used is the SPI 2.4" TFT LCD SHIELD with an ILI9341 controller
