

#ifndef TESTLCDI2C_BSP_H
#define TESTLCDI2C_BSP_H

#include "stm32f4xx_hal.h"

//#define CoveringDriverMotor_ENA_Pin GPIO_PIN_4
#define CoveringDriverMotor_ENA_GPIO_Port GPIOD

#define CoveringDriverMotor_IN1_Pin GPIO_PIN_3
#define CoveringDriverMotor_IN1_GPIO_Port GPIOD

#define CoveringDriverMotor_IN2_Pin GPIO_PIN_2
#define CoveringDriverMotor_IN2_GPIO_Port GPIOD

#define ValveDriver_ENA_Pin GPIO_PIN_12
#define ValveDriver_ENA_GPIO_Port GPIOC

#define ValveDriver_IN1_Pin GPIO_PIN_11
#define ValveDriver_IN1_GPIO_Port GPIOC

#define ValveDriver_IN2_Pin GPIO_PIN_10
#define ValveDriver_IN2_GPIO_Port GPIOC
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
	uint8_t soilHumidity;

}Output_DataTypeDef;


void BSP_Init(void);
void BSP_Delay(uint16_t);
void BSP_LCD_Temperature(uint8_t);
void BSP_LCD_Humidity(uint8_t);
void BSP_LCD_SoilHumidity(uint8_t);
uint8_t BSP_Get_percentageHS(uint16_t);
void BSP_Detect_Movement();
void BSP_CoverFromTemperature(int estado_cortina, int cortina_manual);
uint8_t BSP_SoilHumidity();
void BSP_Irrigation(int rangohmin, int rangohmax);
void BSP_TurnOn_Valve();
void BSP_TurnOff_Valve();
void BSP_Keypad(int tecla, uint32_t rangohmin, uint32_t rangohmax, int estado_cortina, int cortina_manual);

void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void BSP_Output_Init(void);
void BSP_Output_Toggle(void *output);
void BSP_Output_On(void *output);
void BSP_Output_Off(void *output);
#endif //TESTLCDI2C_BSP_H










