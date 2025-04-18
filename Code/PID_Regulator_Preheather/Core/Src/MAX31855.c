#include"MAX31855.h"
//SPI_HandleTypeDef hspi1;

// ------------------- Variables ----------------

uint8_t Error=0;                                      // Thermocouple Connection acknowledge Flag
uint32_t sign=0;									  // Sign bit
uint8_t DATARX[4];                                    // Raw Data from MAX6675
// ------------------- Functions ----------------
float Max31855_Read_Temp(int sensor){
int Temp=0;                                           // Temperature Variable
if(sensor == 1 )
{
	HAL_GPIO_WritePin(SSPORT1,SSPIN1,GPIO_PIN_RESET);// Low State for SPI Communication
	HAL_SPI_Receive(&hspi1,DATARX,4,1000);                // DATA Transfer
	HAL_GPIO_WritePin(SSPORT1,SSPIN1,GPIO_PIN_SET);		  // High State for SPI Communication
}
else if(sensor==2 ){
	HAL_GPIO_WritePin(SSPORT2,SSPIN2,GPIO_PIN_RESET);// Low State for SPI Communication
	HAL_SPI_Receive(&hspi1,DATARX,4,1000);                // DATA Transfer
	HAL_GPIO_WritePin(SSPORT2,SSPIN2,GPIO_PIN_SET);
}
Error=DATARX[3]&0x07;								  // Error Detection
sign=(DATARX[0]&(0x80))>>7;							  // Sign Bit calculation

if(DATARX[3] & 0x07)								  // Returns Error Number
return(-1*(DATARX[3] & 0x07));

else if(sign==1){									  // Negative Temperature
Temp = (DATARX[0] << 6) | (DATARX[1] >> 2);
Temp&=0b01111111111111;
Temp^=0b01111111111111;
return((double)-Temp/4);
	}

else												  // Positive Temperature
{
		Temp = (DATARX[0] << 6) | (DATARX[1] >> 2);
		return((double)Temp / 4);
}
}
