
#include "main.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "bsp.h"
#include "lcd_i2cModule.h"
#include "DHT.h"
#include "keypad.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"  //Eliminar mas tarde
#include "stm32f4xx.h"

QueueHandle_t Queue1Handle;
QueueHandle_t Queue3Handle;
QueueHandle_t Queue5Handle;

void KeypadTask(void const * argument);
void SensorsTask(void const * argument);
void StateMachineTask(void const * argument);

int main(void)
{
  BSP_Init();
  APP_Show_SystemIntro();

  Queue1Handle = xQueueCreate(3, sizeof(LCD_DataTypeDef));
  Queue3Handle = xQueueCreate(3, sizeof(uint8_t));
  Queue5Handle = xQueueCreate(3, sizeof(uint8_t));

  xTaskCreate(KeypadTask, "KeypadTask", 512, NULL, 0, NULL);
  xTaskCreate(SensorsTask, "SensorsTask", 512, NULL, 2, NULL);
  xTaskCreate(StateMachineTask, "UserInterfaceTask", 1280, NULL, 1, NULL);

  vTaskStartScheduler();

  while (1)
  {

  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

void KeypadTask(void const * argument)
{
	uint8_t key;

  for(;;)
  {
	  key = keypad_read();

	if(key != 0){

		xQueueSend(Queue3Handle, &key, 5000);
	}
    vTaskDelay(10);
  }
}

void SensorsTask(void const * argument){

	DHT_DataTypeDef DHT22;
	LCD_DataTypeDef LCD_Data;

	while(1){

		DHT_GetData(&DHT22);
		LCD_Data.humidity = DHT22.Humidity;
		LCD_Data.temperature = DHT22.Temperature;
		LCD_Data.soilHumidity = APP_SoilHumidity();
		xQueueSend(Queue1Handle, &LCD_Data, 0);
		xQueueSend(Queue5Handle, &LCD_Data.soilHumidity, 0);
		vTaskDelay(1100);
	}
}

void StateMachineTask(void const * argument){

	uint8_t rx_key;
	uint8_t state = 0;
	LCD_DataTypeDef LCD_Data;
	uint8_t irrigationHumidity;
	uint8_t rangohmin = 0;
	uint8_t rangohmax = 0;

	while(1){

		xQueueReceive(Queue5Handle, &irrigationHumidity, 0);

			if(state == 6){

			 LCD_SetCursor(1, 1);
			 LCD_Print("                ", 1);
			 LCD_SetCursor(2, 1);
			 LCD_Print("    REGANDO     ", 1);

				if (irrigationHumidity < rangohmin){
					BSP_TurnOn_Valve();
				}
			 	 if (irrigationHumidity >= rangohmax){
					BSP_TurnOff_Valve();
					state = 0;
				 }
			if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE){
				switch (rx_key) {

					case 42: state = 1; BSP_TurnOff_Valve(); break;							 //TECLA 'A'
					case 65: state = 1; BSP_TurnOff_Valve(); break;
					case 67: state = 1; BSP_TurnOff_Valve(); break;
					case 48: state = 6; break;							 //TECLA 'B'
					case 35: state = 6; break;                           //TECLA 'C'
					case 68: state = 6; break;                           //TECLA 'D'

					}
				}
			}

			if(state == 0){
				xQueueReceive(Queue1Handle, &LCD_Data, 0);
				BSP_LCD_Humidity(LCD_Data.humidity);
				BSP_LCD_Temperature(LCD_Data.temperature);
				BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);

				if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE){
					switch (rx_key) {

					            case 42: state = 1; break;							 //TECLA 'A'
					            case 65: state = 1; break;
					            case 67: state = 1; break;
					            case 48: state = 0; break;							 //TECLA 'B'
					            case 35: state = 0; break;                           //TECLA 'C'
					            case 68: state = 0; break;                           //TECLA 'D'

					        }
				}
				if (irrigationHumidity < rangohmin){
					BSP_TurnOn_Valve();
					state = 6;
				}
			 	 if (irrigationHumidity >= rangohmax){
					BSP_TurnOff_Valve();
				 }

			}
			if(state == 1){
                LCD_SetCursor(1, 1);
                LCD_Print("    MINIMO:     ", 1);
				LCD_SetCursor(2, 1);
				LCD_Print("                ", 1);
        		if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE) {
        			switch (rx_key) {
        				case 49: state = 2; rangohmin = 10; break;
        				case 52: state = 2; rangohmin = 20; break;
        				case 55: state = 2; rangohmin = 30; break;
        				case 50: state = 2; rangohmin = 40; break;
        				case 53: state = 2; rangohmin = 50; break;
        				case 56: state = 2; rangohmin = 60; break;
        				case 51: state = 2; rangohmin = 70; break;
        				case 54: state = 2; rangohmin = 80; break;
        				case 57: state = 2; rangohmin = 90; break;
        				case 66: state = 2; rangohmin =  0; break;
        				default: state = 1;
        			}
        		}
			}


			if(state == 2){
                LCD_SetCursor(1, 1);
                LCD_Print("    MINIMO:     ", 1);
				LCD_SetCursor(2, 1);
				LCD_Print("                ", 1);
        		if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE) {
        			switch (rx_key) {
        				case 49: state = 3; rangohmin = rangohmin + 1; break;
        				case 52: state = 3; rangohmin = rangohmin + 2; break;
        				case 55: state = 3; rangohmin = rangohmin + 3; break;
        				case 50: state = 3; rangohmin = rangohmin + 4; break;
        				case 53: state = 3; rangohmin = rangohmin + 5; break;
        				case 56: state = 3; rangohmin = rangohmin + 6; break;
        				case 51: state = 3; rangohmin = rangohmin + 7; break;
        				case 54: state = 3; rangohmin = rangohmin + 8; break;
        				case 57: state = 3; rangohmin = rangohmin + 9; break;
        				case 66: state = 3; break;
        			}
        		}
        		if(rangohmin == 99){
        			state = 2;
        			rangohmin = rangohmin - 9;
        		}
			}

			if(state == 3){
                LCD_SetCursor(1, 1);
                LCD_Print("    MINIMO:%1u   ", rangohmin);
                LCD_SetCursor(2, 1);
                LCD_Print("    MAXIMO:     ", 1);
                if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE) {
    			switch (rx_key) {
    				case 66:
    					if (rangohmin < 10){
    						rangohmax = 0;
    						state = 4;
    					}
    				break;
    				case 49:
    					if (rangohmin < 20){
    						rangohmax = 10;
    						state = 4;
    					}
    				break;
    				case 52:
    					if (rangohmin < 30){
    						rangohmax = 20;
							state = 4;
    					}
    				break;
    				case 55:
    					if (rangohmin < 40){
    						rangohmax = 30;
    						state = 4;
    					}
    				break;
    				case 50:
    					if (rangohmin < 50){
    						rangohmax = 40;
    						state = 4;
    					}
    				break;
    				case 53:
    					if (rangohmin < 60){
    						rangohmax = 50;
    						state = 4;
    					}
    				break;
    				case 56:
    					if (rangohmin < 70){
    						rangohmax = 60;
    						state = 4;
    					}
    				break;
    				case 51:
    					if (rangohmin < 80){
    						rangohmax = 70;
    						state = 4;
    					}
    				break;
    				case 54:
    					if (rangohmin < 90){
    						rangohmax = 80;
    					    state = 4;
    					 }
    				break;
    				case 57:
    					if (rangohmin < 100){
    						rangohmax = 90;
    					    state = 4;
    					 }
    				break;
    				}
    			}
			}


			if(state == 4){
                LCD_SetCursor(1, 1);
                LCD_Print("    MINIMO:%1u   ", rangohmin);
                LCD_SetCursor(2, 1);
                LCD_Print("    MAXIMO:     ", 1);
                if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE) {

    			switch (rx_key) {
					case 66:
						if (rangohmin < rangohmax){
							rangohmax = rangohmax + 0;
							state = 5;
					}
					break;
    				case 49:
    					if (rangohmin < rangohmax + 1){
    						rangohmax = rangohmax + 1;
    						state = 5;
    					}
    				break;
    				case 52:
    					if (rangohmin < rangohmax + 2){
    						rangohmax = rangohmax + 2;
							state = 5;
    					}
    				break;
    				case 55:
    					if (rangohmin < rangohmax + 3){
    						rangohmax = rangohmax + 3;
    						state = 5;
    					}
    				break;
    				case 50:
    					if (rangohmin < rangohmax + 4){
    						rangohmax = rangohmax + 4;
    						state = 5;
    					}
    				break;
    				case 53:
    					if (rangohmin < rangohmax + 5){
    						rangohmax = rangohmax + 5;
    						state = 5;
    					}
    				break;
    				case 56:
    					if (rangohmin < rangohmax + 6){
    						rangohmax = rangohmax + 6;
    						state = 5;
    					}
    				break;
    				case 51:
    					if (rangohmin < rangohmax + 7){
    						rangohmax = rangohmax + 7;
    						state = 5;
    					}
    				break;
    				case 54:
    					if (rangohmin < rangohmax + 8){
    					    rangohmax = rangohmax + 8;
    					    state = 5;
    					 }
    				break;
    				case 57:
    					if (rangohmin < rangohmax + 9){
    					    rangohmax = rangohmax + 9;
    					    state = 5;
    					 }
    				break;

    				}
    			}
			}

			if(state == 5){
				LCD_SetCursor(2, 1);
				LCD_Print("    MAXIMO:%1u   ", rangohmax);
				HAL_Delay(1000);
				state = 0;
			}
		vTaskDelay(800);
	}
}

void APP_Show_SystemIntro(){
    LCD_i2cDeviceCheck();
    LCD_BackLight(LCD_BL_ON);
    LCD_SetCursor(1,1);
    LCD_Clear();
    LCD_Print("Cargando Datos",1);
    BSP_Delay(2000);
}

void APP_CoverFromTemperature(int estado_cortina, int cortina_manual){
    BSP_CoverFromTemperature(estado_cortina, cortina_manual);
}

uint8_t APP_SoilHumidity(){
	uint8_t SoilHumidity;
	SoilHumidity = BSP_SoilHumidity();
	return SoilHumidity;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
