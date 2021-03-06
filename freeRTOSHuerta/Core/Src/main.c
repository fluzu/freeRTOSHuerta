
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "bsp.h"
#include "lcd_i2cModule.h"
#include "DHT.h"
#include "keypad.h"
#include <stdio.h>


#include "stm32f4xx_hal.h"  //Eliminar mas tarde
#include "stm32f4xx.h"

//void *DriverMotor_ENA;
//void *DriverMotor_IN1;
//void *DriverMotor_IN2;

//void *DriverValve_ENA;
//void *DriverValve_IN1;
//void *DriverValve_IN2;




osThreadId KeypadTaskHandle;
osThreadId SensorsTaskHandle;
osThreadId UserInterfaceTaskHandle;
osThreadId AutomaticControlTaskHandle;



osMessageQId Queue1Handle;
osMessageQId Queue3Handle;
osMessageQId Queue4Handle;
osMessageQId Queue5Handle;

QueueSetHandle_t QueueSetHandle;


void KeypadTask(void const * argument);
void SensorsTask(void const * argument);
void UserInterfaceTask(void const * argument);
void AutomaticControlTask(void const * argument);



int main(void)
{

    BSP_Init();
    APP_Show_SystemIntro();


  Queue1Handle = xQueueCreate(3, sizeof(LCD_DataTypeDef));
  Queue3Handle = xQueueCreate(3, sizeof(int));
  Queue4Handle = xQueueCreate(3, sizeof(Output_DataTypeDef));
  Queue5Handle = xQueueCreate(3, sizeof(uint32_t));

  QueueSetHandle = xQueueCreateSet(6); // revisar si no es mucho

  xQueueAddToSet(Queue1Handle, QueueSetHandle);
  xQueueAddToSet(Queue3Handle, QueueSetHandle);

  osThreadDef(KeypadTask, KeypadTask, osPriorityLow, 0, 128);
  KeypadTaskHandle = osThreadCreate(osThread(KeypadTask), NULL);


  osThreadDef(SensorsTask, SensorsTask, osPriorityBelowNormal, 0, 128);
  SensorsTaskHandle = osThreadCreate(osThread(SensorsTask), NULL);

  osThreadDef(UserInterfaceTask, UserInterfaceTask, osPriorityNormal, 0, 128);
  UserInterfaceTaskHandle = osThreadCreate(osThread(UserInterfaceTask), NULL);

  osThreadDef(AutomaticControlTask, AutomaticControlTask, osPriorityHigh, 0, 128);
  AutomaticControlTaskHandle = osThreadCreate(osThread(AutomaticControlTask), NULL);



  osKernelStart();


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
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
 // if (htim->Instance == TIM5) {
 //   HAL_IncTick();
 // }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
//}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void KeypadTask(void const * argument)
{

	 int key;

  for(;;)
  {
	  key = keypad_read();

	if(key != 0){

		xQueueSend(Queue3Handle, &key, 5000);  // sacar pormax_delay que es puro bloqueante
	}

    osDelay(10);


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


		osDelay(1000);                //Bajar para testear
	}
}

void UserInterfaceTask(void const * argument){

	int rx_key;
	LCD_DataTypeDef LCD_Data;
	Output_DataTypeDef Output_Data;
	uint32_t rangohmin = 0;
	uint32_t rangohmax = 30;
	int estado_cortina;
	int cortina_manual;

	while(1){


		QueueSetMemberHandle_t who_unblocked = xQueueSelectFromSet(QueueSetHandle, 0); //si no es 0 es se rompe con Hard_Falut interrupt
		if(who_unblocked == Queue1Handle){
			if(xQueueReceive(Queue1Handle, &LCD_Data, 0)){

				LCD_Clear();
				BSP_LCD_Humidity(LCD_Data.humidity);
				BSP_LCD_Temperature(LCD_Data.temperature);
				BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);


			}
		}
		else if(who_unblocked == Queue3Handle){
			if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE){
				switch (rx_key) {
				            case 65:
				                LCD_Clear();
				                LCD_SetCursor(1, 5);
				                LCD_Print("MINIMO:", 1);
				                HAL_Delay(2000);

				                who_unblocked = xQueueSelectFromSet(QueueSetHandle, portMAX_DELAY);
				                if(who_unblocked == Queue3Handle){

				                	if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE) {
				                		switch (rx_key) {
				                			case 49: rangohmin = 10; break;
				                			case 50: rangohmin = 20; break;
				                			case 51: rangohmin = 30; break;
				                			case 52: rangohmin = 40; break;
				                			case 53: rangohmin = 50; break;
				                			case 54: rangohmin = 60; break;
				                			case 55: rangohmin = 70; break;
				                			case 56: rangohmin = 80; break;
				                			case 57: rangohmin = 90; break;
				                			case 48: rangohmin =  0; break;
				                			default: rangohmin = 100;              //FALTA CASO 100
				                		}
				                	}//revisar corchete
				                }
				                LCD_SetCursor(1, 5);
				                LCD_Print("MINIMO:%1u", rangohmin);
				                LCD_SetCursor(2, 5);
				                LCD_Print("MAXIMO:", 1);


				                who_unblocked = xQueueSelectFromSet(QueueSetHandle, portMAX_DELAY);
				                if(who_unblocked == Queue3Handle){

				                	if(xQueueReceive(Queue3Handle, &rx_key, 0) == pdTRUE) {
				                		switch (rx_key) {
				                			case 49: rangohmax = 10; break;
				                			case 50: rangohmax = 20; break;
				                			case 51: rangohmax = 30; break;
				                			case 52: rangohmax = 40; break;
				                			case 53: rangohmax = 50; break;
				                			case 54: rangohmax = 60; break;
				                			case 55: rangohmax = 70; break;
				                			case 56: rangohmax = 80; break;
				                			case 57: rangohmax = 90; break;
				                			case 48: rangohmax =  0; break;
				                			default: rangohmax = 100;
				                		}                                           //FALTA CASO ERROR QUE SEA MENOR AL M??NIMO
				                	}
				                }
				                LCD_SetCursor(2, 5);
				                LCD_Print("MAXIMO:%1u", rangohmax);
				                HAL_Delay(4000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				               	BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);

				                break;
				            case 49:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 1", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				               	BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 50:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 2", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 51:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 3", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 52:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 4", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 53:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 5", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 54:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 6", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 66:                                //TECLA 'B'
				//                LCD_SetCursor(1, 1);
				//                LCD_Send_String("Ingrese 1 por AM/2 por PM", STR_NOSLIDE);
				//                do {
				//                    tecla = keypad_read();
				//                    switch (tecla) {
				//                        case 0:             break;   //buen metodo?
				//                        case 49: AMoPM = 1; break;
				//                        case 50: AMoPM = 2; break;
				//                        default:
				//                            LCD_Clear();
				//                            LCD_SetCursor(1, 1);
				//                           HAL_Delay(3000);
				//                            LCD_Clear();
				//                            LCD_SetCursor(1, 1);
				//                            LCD_Send_String("Ingrese 1 por AM/2 por PM", STR_NOSLIDE);
				//                    }
				//                } while (AMoPM != 1 && AMoPM != 2);
				//                if (AMoPM == 1){
				//                    LCD_SetCursor(1, 1);
				//                    LCD_Send_String("Ingrese hora de riego", STR_NOSLIDE);
				//                    do {
				//                        tecla = keypad_read();
				//                        switch (tecla) {
				//                            case 49:
				//                                                                  //PROBLEMA
				//                                hora_de_riego = 1;
				//                                break;
				//                            case 50: hora_de_riego = 2; break;
				//                            case 51: hora_de_riego = 3; break;
				//                            case 52: hora_de_riego = 4; break;
				//                            case 53: hora_de_riego = 5; break;
				//                            case 54: hora_de_riego = 6; break;
				//                            case 55: hora_de_riego = 7; break;
				//                            case 56: hora_de_riego = 8; break;
				//                            case 57: hora_de_riego = 9; break;
				                //case 48: hora_de_riego =  0; break;
				//                        }
				//                    } while (tecla == 0);
				//                }
				//                if (AMoPM == 2){

				//                }
				                break;
				            case 55:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 7", 1);
				                HAL_Delay(2000);

				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 56:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 8", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 57:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 9", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 67:                                             //TECLA 'C'
				            	LCD_Clear();
				                LCD_SetCursor(2, 5);
				                LCD_Print("PESTICIDA", 1);
				              //  htim2.Instance->CCR1 = 75; //ANGULO 90 GRADOS
				                HAL_Delay(4000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				            case 68:                                             //TECLA 'D'
				                if(estado_cortina == 0) {       //flag para ver si la cortina esta abierta o cerrada
				                    LCD_Clear();
				                	LCD_SetCursor(2, 1);
				                    LCD_Print("CERRANDO CORTINA", 1);
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //  IN1
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //  IN2
				                    while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));   //espera hasta que la cortina toque fin de carrera con pull up
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA
				                    estado_cortina = 1;                                                  //cambio de estado
				                    if (cortina_manual == 0)   //revisar
				                        cortina_manual = 1;        //bandera para saber si se quiere de manera manual la cortina abierta
				                    else
				                        cortina_manual = 0;
				                }
				                else {
				                	LCD_Clear();
				                    LCD_SetCursor(2, 1);
				                    LCD_Print("ABRIENDO CORTINA", 1);
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //  IN1
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //  IN2
				                    while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3));   //espera hasta que la cortina toque fin de carrera con pull up
				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA
				                    estado_cortina = 0;                                           //cambio de estado
				                    if (cortina_manual == 0)   //revisar
				                        cortina_manual = 1;
				                    else
				                        cortina_manual = 0;    //bandera para saber si se quiere de manera manual la cortina abierta
				                }
				                break;
				            case 48:
				            	LCD_Clear();
				                LCD_SetCursor(2, 1);
				                LCD_Print("Ingreso 0", 1);
				                HAL_Delay(2000);
				                LCD_Clear();

				                BSP_LCD_Humidity(LCD_Data.humidity);
				                BSP_LCD_Temperature(LCD_Data.temperature);
				                BSP_LCD_SoilHumidity(LCD_Data.soilHumidity);
				                break;
				        }
			}
		}

		Output_Data.rangohmin = rangohmin;
		Output_Data.rangohmax = rangohmax;
		Output_Data.soilHumidity = LCD_Data.soilHumidity;

		xQueueSend(Queue4Handle, &Output_Data, 3000);


		osDelay(5000);

	}

}

void AutomaticControlTask(void const * argument){

	Output_DataTypeDef Output_Data;
	uint32_t irrigationHumidity;

	while(1){
		if(xQueueReceive(Queue4Handle, &Output_Data, portMAX_DELAY) == pdTRUE){



					if (Output_Data.soilHumidity < Output_Data.rangohmax && Output_Data.soilHumidity > Output_Data.rangohmin){


						LCD_Clear();

						do {

			            	LCD_SetCursor(2, 5);
							LCD_Print("REGANDO", 1);
							BSP_TurnOn_Valve();
							xQueueReceive(Queue5Handle, &irrigationHumidity, 2000);

						}
						while (irrigationHumidity <= Output_Data.rangohmax && irrigationHumidity >= Output_Data.rangohmin);
						BSP_TurnOff_Valve();
						LCD_Clear();
					}



		}
	}
}




void APP_Timer10ms(){ //Borrar

}
void APP_Timer100ms(){

}
void APP_Timer1000ms(){

}
void APP_Timer10s(){

}
void APP_Show_SystemIntro(){
    LCD_i2cDeviceCheck();
    LCD_BackLight(LCD_BL_ON);
    LCD_SetCursor(1,1);
    LCD_Clear();
    LCD_Print("Cargando Datos",1);
    BSP_Delay(2000);
    //LCD_Clear();
}


void APP_Show_Movement(){
    BSP_Detect_Movement();
}

void APP_CoverFromTemperature(int estado_cortina, int cortina_manual){
    BSP_CoverFromTemperature(estado_cortina, cortina_manual);
}

uint32_t APP_SoilHumidity(){
	int SoilHumidity;
	SoilHumidity = BSP_SoilHumidity();
	return SoilHumidity;
}

void APP_Irrigation(int rangohmin, int rangohmax){
    BSP_Irrigation(rangohmin, rangohmax);
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
