#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"



void APP_Timer10ms();
void APP_Timer100ms();
void APP_Timer1000ms();
void APP_Timer10s();

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void Error_Handler(void);

void APP_Show_SystemIntro();
void APP_Keypad(int rangohmin, int rangohmax, int estado_cortina, int cortina_manual);
void APP_Show_DHT22();
void APP_Show_Movement();
void APP_CoverFromTemperature(int estado_cortina, int cortina_manual);
uint32_t APP_SoilHumidity();
void APP_Irrigation(int rangohmin, int rangohmax);

void Error_Handler(void);

#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
