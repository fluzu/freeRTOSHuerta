/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "bsp.h"
#include "lcd_i2cModule.h"
#include "DHT.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
void *DriverMotor_ENA;
void *DriverMotor_IN1;
void *DriverMotor_IN2;

void *DriverValve_ENA;
void *DriverValve_IN1;
void *DriverValve_IN2;


extern DHT_DataTypeDef DHT22;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    BSP_Init();
    APP_Show_SystemIntro();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  int estado_cortina = 0;
  int cortina_manual = 0;        //bandera si se presiona de manera manual la cortina
  int rangohmin = 50;
  int rangohmax = 60;            //REVISAR RANGO INICIAL DE HUMEDAD
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      //htim2.Instance->CCR1 = 50; //ANGULO 45 GRADOS REVISAR SI NO ROME SERVO ESTANDO EN BUCLE
///Teclado
      APP_Keypad(rangohmin, rangohmax, estado_cortina, cortina_manual);
///DHT22
      APP_Show_DHT22();
///Sensor humedad de suelo
      APP_Show_SoilHumidity();
      ///Sensor movimiento
      APP_Show_Movement();
///Cerrar o abrir cortina por temperatura
      APP_CoverFromTemperature(estado_cortina, cortina_manual);
///Valvula solenoide riego
      APP_Irrigation(rangohmin, rangohmax);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}




void APP_Timer10ms(){

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

void APP_Show_DHT22(){
    LCD_Clear();  //REVISAR necesidad de esta funcion
    DHT_GetData(&DHT22);
    BSP_LCD_Temperature(DHT22.Temperature);
    BSP_LCD_Humidity(DHT22.Humidity);
}

void APP_Show_Movement(){
    BSP_Detect_Movement();
}

void APP_CoverFromTemperature(int estado_cortina, int cortina_manual){
    BSP_CoverFromTemperature(estado_cortina, cortina_manual);
}

void APP_Show_SoilHumidity(){
    BSP_Show_SoilHumidity();
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