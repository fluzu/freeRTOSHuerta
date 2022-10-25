

#ifndef TESTLCDI2C_BSP_H
#define TESTLCDI2C_BSP_H

#include "stm32f4xx_hal.h"

typedef struct
{
	uint32_t temperature;
	uint32_t humidity;
	uint32_t soilHumidity;

}LCD_DataTypeDef;

typedef struct
{
	uint32_t rangohmin;
	uint32_t rangohmax;
	uint32_t soilHumidity;

}Output_DataTypeDef;


void BSP_Init(void);
void BSP_Delay(uint16_t);
void BSP_LCD_Temperature(uint32_t);
void BSP_LCD_Humidity(uint32_t);
void BSP_LCD_SoilHumidity(uint32_t);
uint32_t BSP_Get_percentageHS(uint32_t);

void BSP_CoverFromTemperature(int estado_cortina, int cortina_manual);
uint32_t BSP_SoilHumidity();
void BSP_Irrigation(int rangohmin, int rangohmax);
void BSP_TurnOn_Valve();
void BSP_TurnOff_Valve();

#endif //TESTLCDI2C_BSP_H










