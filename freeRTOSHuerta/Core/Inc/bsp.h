

#ifndef TESTLCDI2C_BSP_H
#define TESTLCDI2C_BSP_H

#include "stm32f4xx_hal.h"

#define LD4_Pin GPIO_PIN_12


typedef struct
{
	uint8_t temperature;
	uint8_t humidity;
	uint8_t soilHumidity;

}LCD_DataTypeDef;

typedef struct
{
	uint8_t rangohmin;
	uint8_t rangohmax;


}Output_DataTypeDef;




void BSP_Init(void);
void BSP_Delay(uint16_t);
void BSP_LCD_Temperature(uint8_t);
void BSP_LCD_Humidity(uint8_t);
void BSP_LCD_SoilHumidity(uint8_t);
uint8_t BSP_Get_percentageHS(uint16_t);

void BSP_CoverFromTemperature(int estado_cortina, int cortina_manual);
uint8_t BSP_SoilHumidity();
//void BSP_Irrigation(int rangohmin, int rangohmax);
void BSP_TurnOn_Valve();
void BSP_TurnOff_Valve();


void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


#endif //TESTLCDI2C_BSP_H










