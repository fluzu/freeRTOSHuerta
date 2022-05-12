
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "bsp.h"
#include "lcd_i2cModule.h"
#include "DHT.h"
#include "keypad.h"


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
osMessageQId Queue2Handle;
osMessageQId Queue3Handle;



void KeypadTask(void const * argument);
void SensorsTask(void const * argument);
void UserInterfaceTask(void const * argument);
void AutomaticControlTask(void const * argument);



int main(void)
{

    BSP_Init();
    APP_Show_SystemIntro();


  Queue1Handle = xQueueCreate(4, sizeof(float));
  Queue2Handle = xQueueCreate(4, sizeof(uint32_t));
  Queue3Handle = xQueueCreate(4, sizeof(int));

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

		xQueueSend(Queue3Handle, &key, 1000);
	}

    osDelay(10);


  }

}

void SensorsTask(void const * argument){

	DHT_DataTypeDef DHT22;
	float tx_temperature;
	float tx_humidity;
	uint32_t soilHumidity;

	while(1){

		DHT_GetData(&DHT22);
		tx_humidity = DHT22.Humidity;
		tx_temperature = DHT22.Temperature;

		soilHumidity = APP_SoilHumidity();

		osDelay(2000);                //Bajar para testear

		xQueueSend(Queue1Handle, &tx_humidity, 2000);
		xQueueSend(Queue1Handle, &tx_temperature, 1000);

		xQueueSend(Queue2Handle, &soilHumidity, 2000);
	}
}

void UserInterfaceTask(void const * argument){


	uint32_t soilHumidity;
	int rx_key;
	float rx_temperature;
	float rx_humidity;

	while(1){

		xQueueReceive(Queue3Handle, &rx_key, 2000);

		if(xQueueReceive(Queue1Handle, &rx_humidity, 2000) == pdTRUE){
			if(xQueueReceive(Queue1Handle, &rx_temperature, 2000 == pdTRUE)){
				if(xQueueReceive(Queue2Handle, &soilHumidity, 2000 == pdTRUE)){

					BSP_LCD_Temperature(rx_temperature);
					BSP_LCD_Humidity(rx_humidity);
					BSP_LCD_SoilHumidity(soilHumidity);

				}
			}
		}



		osDelay(1000);
	}
}

void AutomaticControlTask(void const * argument){

//	int rangohmin;				//Hace falta poner static???????????????????????
//	int rangohmax;

	while(1){
//		xQueueReceive(Queue1Handle, &rangohmin, 1000);
//		xQueueReceive(Queue1Handle, &rangohmax, 1000);
//		//APP_Irrigation(rangohmin, rangohmax);   //Funciona valor recibido por la queue???????????????
		osDelay(500);
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
    BSP_Delay(4000);
    LCD_Clear();
}

void APP_Keypad(int rangohmin, int rangohmax, int estado_cortina, int cortina_manual){
    BSP_Keypad(rangohmin, rangohmax, estado_cortina, cortina_manual);
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
