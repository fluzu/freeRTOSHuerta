#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "lcd_i2cModule.h"
#include "Timer_Delay.h"
#include "DHT.h"
#include "keypad.h"
#include "bsp.h"
#include "main.h"
#include <stdio.h>


static DHT_DataTypeDef DHT22; //Revisar si trae problemas static

static ADC_HandleTypeDef hadc1;

static TIM_HandleTypeDef htim2;


void BSP_Init() {

    HAL_Init();
    TimerDelay_Init();
    LCD_Init();
    keypad_init();
    //BSP_Output_Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Init PWM

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); //  ENA2
}

void BSP_Delay(uint16_t Delay){
    delay_ms(Delay);
}

void BSP_LCD_Temperature(uint32_t temperatura) {
    LCD_SetCursor(1, 4);
    LCD_Print("Grados:%1uC", temperatura);
}

void BSP_LCD_Humidity(uint32_t humedad) {
    LCD_SetCursor(2, 1);
    LCD_Print("HA:%1u%%", humedad);
}

void BSP_LCD_SoilHumidity(uint32_t soilHumidity){
	LCD_SetCursor(2, 10);
	LCD_Print("HS:%1u%%", soilHumidity);
}


uint32_t BSP_Get_percentageHS(uint32_t value){
    int hummin = 4095;                      //REVISAR TIPO DE DATO
    int hummax = 2300;                      //REVISAR Maximo y Minimo en especial maximo
//#define humminp = 0
    int hummaxp = 100;
    if (value > hummin)
        value = hummin;
    if (value <= hummax)
        value = hummax + 1;
    value = value - hummax;
    return 100 - ((value * hummaxp) / (hummin - hummax));
}


void BSP_CoverFromTemperature(int estado_cortina, int cortina_manual){
    if(DHT22.Temperature < 6) {           //se puede optimizar preguntando con dos condiciones?
        if (estado_cortina == 0 && cortina_manual == 0)
        {        //flag para ver si la cortina esta abierta o cerrada  REVISAR cortina manual
            LCD_Clear();
            LCD_SetCursor(2, 1);
            LCD_Print("CERRANDO CORTINA", 1);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //  IN1
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //  IN2
            while ( !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));   //espera hasta que la cortina toque fin de carrera                                              //VER CUANTO TIEMPO DEMORA EN CERRAR CORTINA
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA
            estado_cortina = 1;                                                  //cambio de estado
        }
    }
    else{
        if (estado_cortina == 1 && cortina_manual == 0) {
            LCD_Clear();
            LCD_SetCursor(2, 1);
            LCD_Print("ABRIENDO CORTINA", 1);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);       //  ENA
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);     //  IN1
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);       //  IN2
            while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3));   //espera hasta que la cortina toque fin de carrera                                                  //VER CUANTO TIEMPO DEMORA EN ABRIR CORTINA
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);     //  ENA
            estado_cortina = 0;                                                    //cambio de estado
        }
    }
}

uint32_t BSP_SoilHumidity(){
	uint32_t value_adc[3];
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){     //incilur esta parte en el solenoide para hecr while?
        value_adc[0] = HAL_ADC_GetValue(&hadc1);
        value_adc[0] = BSP_Get_percentageHS(value_adc[0]);
        HAL_ADC_Stop(&hadc1);
    }
    return value_adc[0];
}

void BSP_Irrigation(int rangohmin, int rangohmax){
	uint32_t value_adc[3];

                //REVISAR RANGO INICIAL DE HUMEDAD
    LCD_Clear();
    do {                                  //ver caso si se quiere regar durante movimiento
        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
            value_adc[0] = HAL_ADC_GetValue(&hadc1);
            value_adc[0] = BSP_Get_percentageHS(value_adc[0]);
            HAL_ADC_Stop(&hadc1);
        }
        if (value_adc[0] < rangohmax && value_adc[0] > rangohmin) {//revisar hacer con while
            LCD_SetCursor(2, 5);
            LCD_Print("REGANDO", 1);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   //  ENA
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   //  IN1
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); //  IN2

        }
    }
    while (value_adc[0] <= rangohmax && value_adc[0] >= rangohmin);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); //  ENA
}

void BSP_TurnOn_Valve(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   //  ENA
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   //  IN1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); //  IN2
}

void BSP_TurnOff_Valve(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); //  ENA
}
