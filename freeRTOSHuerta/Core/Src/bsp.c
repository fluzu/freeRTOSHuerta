#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "lcd_i2cModule.h"
#include "Timer_Delay.h"
#include "DHT.h"
#include "keypad.h"
#include "bsp.h"
#include "main.h"
#include <stdio.h>



ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;




void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);




void ADC_Select_CH1 (void){

		ADC_ChannelConfTypeDef sConfig = {0};
	    sConfig.Channel = ADC_CHANNEL_1;
	    sConfig.Rank = 1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	        Error_Handler();
	    }

}

void ADC_Select_CH3 (void){

		ADC_ChannelConfTypeDef sConfig = {0};
	    sConfig.Channel = ADC_CHANNEL_3;
	    sConfig.Rank = 1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	        Error_Handler();
	    }
}

void ADC_Select_CH4 (void){

		ADC_ChannelConfTypeDef sConfig = {0};
	    sConfig.Channel = ADC_CHANNEL_4;
	    sConfig.Rank = 1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	        Error_Handler();
	    }

}

void BSP_Init() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    TimerDelay_Init();
    LCD_Init();
    keypad_init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 					 // Init PWM

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); 		 //  ENA1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);		 //  ENA2
}

void BSP_Delay(uint16_t Delay){
    delay_ms(Delay);
}

void BSP_LCD_Temperature(uint8_t temperature) {
    LCD_SetCursor(1, 1);
    LCD_Print("   Grados:%1uC   ", temperature);
}

void BSP_LCD_Humidity(uint8_t humidity) {
    LCD_SetCursor(2, 1);
    LCD_Print("HA:%1u%%   ", humidity);
}

void BSP_LCD_SoilHumidity(uint8_t soilHumidity){
	LCD_SetCursor(2, 10);
	LCD_Print("HS:%1u%%  ", soilHumidity);
}


uint8_t BSP_Get_percentageHS(uint16_t value){
    int hummin = 4095;
    int hummax = 2300;
//#define humminp = 0
    int hummaxp = 100;
    if (value > hummin)
        value = hummin;
    if (value <= hummax)
        value = hummax + 1;
    value = value - hummax;
    return 100 - ((value * hummaxp) / (hummin - hummax));
}

//void BSP_CoverFromTemperature(int estado_cortina, int cortina_manual){
//    if(DHT22.Temperature < 6) {           //se puede optimizar preguntando con dos condiciones?
//        if (estado_cortina == 0 && cortina_manual == 0)
//        {        //flag para ver si la cortina esta abierta o cerrada  REVISAR cortina manual
//            LCD_Clear();
//            LCD_SetCursor(2, 1);
//            LCD_Print("CERRANDO CORTINA", 1);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); //  ENA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //  IN1
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //  IN2
//            while ( !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5));   //espera hasta que la cortina toque fin de carrera                                              //VER CUANTO TIEMPO DEMORA EN CERRAR CORTINA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //  ENA
//            estado_cortina = 1;                                                  //cambio de estado
//        }
//    }
//    else{
//        if (estado_cortina == 1 && cortina_manual == 0) {
//            LCD_Clear();
//            LCD_SetCursor(2, 1);
//            LCD_Print("ABRIENDO CORTINA", 1);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);       //  ENA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);     //  IN1
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);       //  IN2
//            while (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3));   //espera hasta que la cortina toque fin de carrera                                                  //VER CUANTO TIEMPO DEMORA EN ABRIR CORTINA
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);     //  ENA
//            estado_cortina = 0;                                                    //cambio de estado
//        }
//    }
//}

uint8_t BSP_SoilHumidity(){
	uint16_t value_adc[3];
	uint8_t average;
	ADC_Select_CH1();
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
        value_adc[0] = HAL_ADC_GetValue(&hadc1);
        value_adc[0] = BSP_Get_percentageHS(value_adc[0]);
        HAL_ADC_Stop(&hadc1);
    }
//	ADC_Select_CH3();
//    HAL_ADC_Start(&hadc1);
//    if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
//        value_adc[1] = HAL_ADC_GetValue(&hadc1);
//        value_adc[1] = BSP_Get_percentageHS(value_adc[1]);
//        HAL_ADC_Stop(&hadc1);
//    }
//	ADC_Select_CH4();
//    HAL_ADC_Start(&hadc1);
//    if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
//        value_adc[2] = HAL_ADC_GetValue(&hadc1);
//        value_adc[2] = BSP_Get_percentageHS(value_adc[2]);
//        HAL_ADC_Stop(&hadc1);
//    }
    average = (value_adc[0] + value_adc[1])/2;
    return value_adc[0];
}

void BSP_TurnOn_Valve(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   //  ENA
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   //  IN1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); //  IN2
}

void BSP_TurnOff_Valve(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); //  ENA
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
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
//    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//    */
//    sConfig.Channel = ADC_CHANNEL_1;
//    sConfig.Rank = 1;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//    */
//    sConfig.Channel = ADC_CHANNEL_3;
//    sConfig.Rank = 2;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//    */
//    sConfig.Channel = ADC_CHANNEL_4;
//    sConfig.Rank = 3;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, LD4_Pin|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

    /*Configure GPIO pins : PE3 PE5 */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PC3 PC10 PC11 PC12 */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA0 PA2 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
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

    /*Configure GPIO pins : LD4_Pin PD2 PD3 PD4 */
    GPIO_InitStruct.Pin = LD4_Pin|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PA2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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





