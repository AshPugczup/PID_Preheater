/*************************************************************************************
 Title	 :  MAXIM Integrated MAX31855 Library for STM32 Using HAL Libraries
 Author  :  Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Software:  STM32CubeIDE
 Hardware:  Any STM32 device
*************************************************************************************/
#ifndef MAX31855_H_
#define MAX31855_H_
#include "main.h"
#include "stm32f0xx_hal.h"



// ------------------------- Defines -------------------------
extern uint8_t Error;	   // Error Detection - 1-> No Connection / 2-> Short to GND / 4-> Short to VCC
extern SPI_HandleTypeDef hspi1;
#define SSPORT1 GPIOA       // GPIO Port of Chip Select(Slave Select)
#define SSPIN1  GPIO_PIN_4  // GPIO PIN of Chip Select(Slave Select)
#define SSPORT2 GPIOA       // GPIO Port of Chip Select(Slave Select)
#define SSPIN2  GPIO_PIN_15  // GPIO PIN of Chip Select(Slave Select)
// ------------------------- Functions  ----------------------
float Max31855_Read_Temp(int sensor);
#endif
