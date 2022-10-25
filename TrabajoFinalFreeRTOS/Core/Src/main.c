
#include "main.h"
#include "bsp.h"
#include "cmsis_os.h"
#include "queue.h"
#include "lcd_i2cModule.h"
#include "DHT.h"
#include "keypad.h"
#include <stdio.h>
#include <FreeRTOS.h>

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;






osThreadId AutomaticTaskHandle;
osThreadId InterfaceTaskHandle;
osThreadId TheSensorsTaskHandle;
osThreadId TheKeypadTaskHandle;

osMessageQId Queue0Handle;
osMessageQId Queue1Handle;
osMessageQId Queue2Handle;
osMessageQId Queue3Handle;
osMessageQId Queue4Handle;
osMessageQId Queue5Handle;
osMessageQId Queue6Handle;
osMessageQId Queue7Handle;

QueueSetHandle_t QueueSetHandle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void AutomaticControlTask(void const * argument);
void UserInterfaceTask(void const * argument);
void SensorsTask(void const * argument);
void KeypadTask(void const * argument);

int main(void)
{

  BSP_Init();
 // APP_Show_SystemIntro();

  SystemClock_Config();


  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  osMessageQDef(Queue0Handle, 3, uint32_t);
  Queue0Handle = osMessageCreate(osMessageQ(Queue0Handle), NULL);

  osMessageQDef(Queue1Handle, 3, uint32_t);
  Queue1Handle = osMessageCreate(osMessageQ(Queue1Handle), NULL);

  osMessageQDef(Queue2Handle, 3, uint32_t);
  Queue2Handle = osMessageCreate(osMessageQ(Queue2Handle), NULL);

  osMessageQDef(Queue3Handle, 3, int);
  Queue3Handle = osMessageCreate(osMessageQ(Queue3Handle), NULL);


  osMessageQDef(Queue4Handle, 3, uint32_t);
  Queue4Handle = osMessageCreate(osMessageQ(Queue4Handle), NULL);


  osMessageQDef(Queue5Handle, 3, uint32_t);
  Queue5Handle = osMessageCreate(osMessageQ(Queue5Handle), NULL);

  osMessageQDef(Queue6Handle, 3, uint32_t);
  Queue6Handle = osMessageCreate(osMessageQ(Queue6Handle), NULL);

  osMessageQDef(Queue7Handle, 3, uint32_t);
  Queue7Handle = osMessageCreate(osMessageQ(Queue7Handle), NULL);

  QueueSetHandle = xQueueCreateSet(6); // revisar si no es mucho

  xQueueAddToSet(Queue1Handle, QueueSetHandle);
  xQueueAddToSet(Queue3Handle, QueueSetHandle);

//  	  xTaskCreate(AutomaticTask, AutomaticControlTask, 1024, NULL, 3, NULL);
//  	xTaskCreate(InterfaceTask, UserInterfaceTask, 1024, NULL, 2, NULL);
//  	xTaskCreate(TheSensorsTask, SensorsTask, 1024, NULL, 1, NULL);
//  	xTaskCreate(TheKeypadTask, KeypadTask, 1024, NULL, 0, NULL);

  osThreadDef(AutomaticTask, AutomaticControlTask, osPriorityHigh, 0, 1024);
  AutomaticTaskHandle = osThreadCreate(osThread(AutomaticTask), NULL);


  osThreadDef(InterfaceTask, UserInterfaceTask, osPriorityNormal, 0, 1024);
  InterfaceTaskHandle = osThreadCreate(osThread(InterfaceTask), NULL);


  osThreadDef(TheSensorsTask, SensorsTask, osPriorityBelowNormal, 0, 1024);
  TheSensorsTaskHandle = osThreadCreate(osThread(TheSensorsTask), NULL);


  osThreadDef(TheKeypadTask, KeypadTask, osPriorityLow, 0, 1024);
  TheKeypadTaskHandle = osThreadCreate(osThread(TheKeypadTask), NULL);


  osKernelStart();

  while (1)
  {

  }

}



/* USER CODE BEGIN Header_AutomaticControlTask */
/**
  * @brief  Function implementing the AutomaticTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AutomaticControlTask */
void AutomaticControlTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	//Output_DataTypeDef Output_Data;
	uint32_t rangohmin;
	uint32_t rangohmax;
	uint32_t irrigationHumidity;
	uint32_t soilHumidity;
  /* Infinite loop */
  for(;;)
  {
		if(xQueueReceive(Queue4Handle, &soilHumidity, portMAX_DELAY) == pdTRUE){
			xQueueReceive(Queue6Handle, &rangohmin, portMAX_DELAY);
			xQueueReceive(Queue7Handle, &rangohmax, portMAX_DELAY);

					if (soilHumidity < rangohmax && soilHumidity > rangohmin){

						LCD_Clear();

						do {

			            	LCD_SetCursor(2, 5);
							LCD_Print("REGANDO", 1);
							BSP_TurnOn_Valve();
							xQueueReceive(Queue5Handle, &irrigationHumidity, 2000);

						}
						while (irrigationHumidity <= rangohmax && irrigationHumidity >= rangohmin);
						BSP_TurnOff_Valve();
						LCD_Clear();
					}
		}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UserInterfaceTask */
/**
* @brief Function implementing the InterfaceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UserInterfaceTask */
void UserInterfaceTask(void const * argument)
{
  /* USER CODE BEGIN UserInterfaceTask */
	int rx_key;
		//LCD_DataTypeDef LCD_Data;
		//Output_DataTypeDef Output_Data;
		uint32_t rangohmin = 0;
		uint32_t rangohmax = 30;
		uint32_t temperature = 0;
		uint32_t humidity = 0;
		uint32_t soilHumidity = 0;
		int estado_cortina;
		int cortina_manual;
  /* Infinite loop */
  for(;;)
  {
	  QueueSetMemberHandle_t who_unblocked = xQueueSelectFromSet(QueueSetHandle, 0); //si no es 0 es se rompe con Hard_Falut interrupt
	  		if(who_unblocked == Queue1Handle){
	  			if(xQueueReceive(Queue1Handle, &soilHumidity, 0)){
	  				xQueueReceive(Queue0Handle, &temperature, 0);
					xQueueReceive(Queue2Handle, &humidity, 0);

	  				LCD_Clear();
	  				BSP_LCD_Humidity(humidity);
	  				BSP_LCD_Temperature(temperature);
	  				BSP_LCD_SoilHumidity(soilHumidity);

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
	  				                			default: rangohmin = 100;                                   //FALTA CASO 100
	  				                		}
	  				                	}                                                                   //revisar corchete
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
	  				                		}                                                                 //FALTA CASO ERROR QUE SEA MENOR AL MÍNIMO
	  				                	}
	  				                }
	  				                LCD_SetCursor(2, 5);
	  				                LCD_Print("MAXIMO:%1u", rangohmax);
	  				                HAL_Delay(4000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				               	BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);

	  				                break;
	  				            case 49:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 1", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				               	BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 50:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 2", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 51:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 3", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 52:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 4", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 53:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 5", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 54:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 6", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
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

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 56:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 8", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 57:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 9", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 67:                                             //TECLA 'C'
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 5);
	  				                LCD_Print("PESTICIDA", 1);
	  				              //  htim2.Instance->CCR1 = 75; //ANGULO 90 GRADOS
	  				                HAL_Delay(4000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				            case 68:                                             //TECLA 'D'
	  				                if(estado_cortina == 0) {     										  //flag para ver si la cortina esta abierta o cerrada
	  				                    LCD_Clear();
	  				                	LCD_SetCursor(2, 1);
	  				                    LCD_Print("CERRANDO CORTINA", 1);
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); 				//  ENA
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); 				//  IN1
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);				 //  IN2
	  				                    while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));   						//espera hasta que la cortina toque fin de carrera con pull up
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); 				//  ENA
	  				                    estado_cortina = 1;                                                  //cambio de estado
	  				                    if (cortina_manual == 0)  										 //revisar
	  				                        cortina_manual = 1;       									 //bandera para saber si se quiere de manera manual la cortina abierta
	  				                    else
	  				                        cortina_manual = 0;
	  				                }
	  				                else {
	  				                	LCD_Clear();
	  				                    LCD_SetCursor(2, 1);
	  				                    LCD_Print("ABRIENDO CORTINA", 1);
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); 			//  ENA
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); 				//  IN1
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); 			//  IN2
	  				                    while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3));  						 //espera hasta que la cortina toque fin de carrera con pull up
	  				                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); 			//  ENA
	  				                    estado_cortina = 0;                                           //cambio de estado
	  				                    if (cortina_manual == 0)   										//revisar
	  				                        cortina_manual = 1;
	  				                    else
	  				                        cortina_manual = 0;   										 //bandera para saber si se quiere de manera manual la cortina abierta
	  				                }
	  				                break;
	  				            case 48:
	  				            	LCD_Clear();
	  				                LCD_SetCursor(2, 1);
	  				                LCD_Print("Ingreso 0", 1);
	  				                HAL_Delay(2000);
	  				                LCD_Clear();

	  				                BSP_LCD_Humidity(humidity);
	  				                BSP_LCD_Temperature(temperature);
	  				                BSP_LCD_SoilHumidity(soilHumidity);
	  				                break;
	  				        }
	  			}
	  		}
	  		soilHumidity = 3;
	  		//soilHumidity = LCD_Data.soilHumidity;

	  		xQueueSend(Queue4Handle, &soilHumidity, 3000); // revisar como incide el 3000 en el comportamiento
	  		xQueueSend(Queue6Handle, &rangohmin, 3000);
	  		xQueueSend(Queue7Handle, &rangohmax, 3000);

	  		osDelay(5000);
	  	}


  /* USER CODE END UserInterfaceTask */
}

/* USER CODE BEGIN Header_SensorsTask */
/**
* @brief Function implementing the TheSensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorsTask */
void SensorsTask(void const * argument)
{
  /* USER CODE BEGIN SensorsTask */
	DHT_DataTypeDef DHT22;
	uint32_t temperature = 0;
	uint32_t humidity = 0;
	uint32_t soilHumidity;
  /* Infinite loop */
  for(;;)
  {
		DHT_GetData(&DHT22);
		humidity = DHT22.Humidity;
		temperature = DHT22.Temperature;
		soilHumidity = APP_SoilHumidity();

		xQueueSend(Queue1Handle, &soilHumidity, 0);
		xQueueSend(Queue0Handle, &temperature, 0);
		xQueueSend(Queue2Handle, &humidity, 0);


		xQueueSend(Queue5Handle, &soilHumidity, 0);

		osDelay(500);                                                         //Bajar para testear
  }
  /* USER CODE END SensorsTask */
}

/* USER CODE BEGIN Header_KeypadTask */
/**
* @brief Function implementing the TheKeypadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeypadTask */
void KeypadTask(void const * argument)
{
  /* USER CODE BEGIN KeypadTask */
	int key;
  /* Infinite loop */
  for(;;)
  {
	  key = keypad_read();

	  	if(key != 0){

	  		xQueueSend(Queue3Handle, &key, 5000);  // sacar pormax_delay que es puro bloqueante
	  	}
	      osDelay(10);
  }
  /* USER CODE END KeypadTask */
}

void APP_Show_SystemIntro(){
    LCD_i2cDeviceCheck();
    LCD_BackLight(LCD_BL_ON);
    LCD_SetCursor(1,1);
    LCD_Clear();
    LCD_Print("Cargando Datos",1);
    //BSP_Delay(2000);
    //LCD_Clear();
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


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin PC3 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE12 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_MCK_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
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
