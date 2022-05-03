

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


void BSP_Init(void);
void BSP_Delay(uint16_t);
void BSP_LCD_Temperature(float);
void BSP_LCD_Humidity(float);
uint32_t BSP_Get_percentageHS(uint32_t);
void BSP_Detect_Movement();
void BSP_CoverFromTemperature(int estado_cortina, int cortina_manual);
void BSP_Show_SoilHumidity();
void BSP_Irrigation(int rangohmin, int rangohmax);
void BSP_Keypad(int rangohmin, int rangohmax, int estado_cortina, int cortina_manual);

void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void BSP_Output_Init(void);
void BSP_Output_Toggle(void *output);
void BSP_Output_On(void *output);
void BSP_Output_Off(void *output);
#endif //TESTLCDI2C_BSP_H










