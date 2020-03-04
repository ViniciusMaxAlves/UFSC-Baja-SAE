/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PAINEL_BUFFSIZE 21

#define MLX90614_ADDR 0x5A
#define MLX90614_TA 0x06
#define MLX90614_TOBJ 0x07

#define MPU6050_ADDR 0x68
#define MPU6050_ACELEROMETRO 0X3B
#define MPU6050_GIROSCOPIO 0X43
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/*
	informacoesPainel:
		0: rotacao
		1: velocidade
		2: combustivel (atribuido valor 1 quando estiver na reserva)
		3: marcha engatada
		4: barra estabilizadora (atribuido valor 1 quando...)

	infTelemetria
		5: temperatura ambiente lsb
		6: temperatura ambiente msb
		7: temperatura objeto lsb
		8: temperatura objeto msb
		9: g_x msb
		10: g_x lsb
		11: g_y msb
		12: g_y lsb
		13: g_z msb
		14: g_z lsb
		15: a_x msb
		16: a_x lsb
		17: a_y msb
		18: a_y lsb
		19: a_z msb
		20: a_z lsb
 */
uint8_t informacoesPainel[PAINEL_BUFFSIZE];
uint8_t bufTemp[3];
uint8_t bufMPU[6];
uint8_t pulsosRotacao = 0;
uint8_t pulsosVelocidade = 0;
uint8_t marchaEngatada = 0;
uint8_t contadorCapacitivo = 0;
uint8_t temCombustivel = 0;
uint8_t contadorSegundosVel = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void verificaRotacao();
void verificaVelocidade();
void verificaCombustivel();
void entraReserva();
void saiReserva();
void verificaMarchaEngatada();
void temperaturaAmbiente();
void temperaturaObjeto();
void leTemperatura(uint8_t reg);
void leituraGiroscopio();
void leituraAcelerometro();
void leituraRegMPU(uint8_t reg);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  for(int i = 0; i < 21; i++){
	  informacoesPainel[i] = 0;
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  uint8_t Data = 0;
  HAL_I2C_Mem_Write(&hi2c2, (MPU6050_ADDR<<1), 0x6B, 1,&Data, 1, 1000);
  Data = 0x07;
  HAL_I2C_Mem_Write(&hi2c2, (MPU6050_ADDR<<1), 0x19, 1, &Data, 1, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Marcha_Posicao1_Pin */
  GPIO_InitStruct.Pin = Marcha_Posicao1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Marcha_Posicao1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Marcha_Posicao2_Pin Combustivel_Pin Barra_Estab_Pin */
  GPIO_InitStruct.Pin = Marcha_Posicao2_Pin|Combustivel_Pin|Barra_Estab_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Velocidade_Pin Rotacao_Pin */
  GPIO_InitStruct.Pin = Velocidade_Pin|Rotacao_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 if (GPIO_Pin == GPIO_PIN_8) {
	 pulsosVelocidade++;
 }

 if (GPIO_Pin == GPIO_PIN_9) {
	 pulsosRotacao++;
 }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	verificaRotacao();
	verificaVelocidade();
	verificaCombustivel();
	verificaMarchaEngatada();
	informacoesPainel[4] = HAL_GPIO_ReadPin(Barra_Estab_GPIO_Port, GPIO_PIN_10);
	temperaturaAmbiente();
	temperaturaObjeto();
	leituraGiroscopio();
	leituraAcelerometro();
	HAL_UART_Transmit_DMA(&huart2, informacoesPainel, PAINEL_BUFFSIZE);
}

void verificaRotacao(){
	if(pulsosRotacao <= 5){
		informacoesPainel[0] = 0;
	}else{
		if(pulsosRotacao >= 6 && pulsosRotacao <= 11){
			informacoesPainel[0] = ((pulsosRotacao-8)/2) + 1;
		}else{
			if(pulsosRotacao >= 17){
				informacoesPainel[0] = 8;
			}else{
				informacoesPainel[0] = pulsosRotacao - 9;
			}
		}
	}
	pulsosRotacao = 0;
	/*
	if(pulsosRotacao <= 7){
		informacoesPainel[0] = 0;
	}else{
		if(pulsosRotacao >= 8 && pulsosRotacao <= 11){
				 informacoesPainel[0] = 1;
		}else{
			if(pulsosRotacao >= 30){
				informacoesPainel[0] = 8;
			}else{
				informacoesPainel[0] = ((pulsosRotacao-12)/3) + 2;
			}
		}
	}
	*/

	//informacoesPainel[0] = pulsosRotacao;
}

void verificaVelocidade(){
	contadorSegundosVel++;
	if(contadorSegundosVel == 2){
		informacoesPainel[1] = pulsosVelocidade * 1.413675; //velocidade = pulsos * (2*pi/4) * raio[0.25] * tempo[1] * 3,6
		pulsosVelocidade = 0;
		contadorSegundosVel = 0;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}

void verificaCombustivel(){
	if(informacoesPainel[2] == 0){
		entraReserva();
	}else{
		saiReserva();
	}
}

void entraReserva(){
	contadorCapacitivo++;
	if(HAL_GPIO_ReadPin(Combustivel_GPIO_Port, GPIO_PIN_1) == 1){
		temCombustivel++;
	}
	if(contadorCapacitivo == 10){
		if(temCombustivel > 5){
			informacoesPainel[2] = 0;
		}else{
			informacoesPainel[2] = 1;
		}
		temCombustivel = 0;
		contadorCapacitivo = 0;
	}
}

void saiReserva(){
	contadorCapacitivo++;
	if(HAL_GPIO_ReadPin(Combustivel_GPIO_Port, GPIO_PIN_1) == 1){
		temCombustivel++;
	}
	if(contadorCapacitivo == 10){
		if(temCombustivel == 10){
			informacoesPainel[2] = 0;
		}
		temCombustivel = 0;
		contadorCapacitivo = 0;
	}
}

void verificaMarchaEngatada(){
	if(HAL_GPIO_ReadPin(Marcha_Posicao1_GPIO_Port, GPIO_PIN_15) == 1 &&
			HAL_GPIO_ReadPin(Marcha_Posicao2_GPIO_Port, GPIO_PIN_0) == 1){
		informacoesPainel[3] = 3;
	}else{
		if(HAL_GPIO_ReadPin(Marcha_Posicao1_GPIO_Port, GPIO_PIN_15) == 1){
			informacoesPainel[3] = 1;
		}else{
			if(HAL_GPIO_ReadPin(Marcha_Posicao2_GPIO_Port, GPIO_PIN_0) == 1){
				informacoesPainel[3] = 2;
			}else{
				informacoesPainel[3] = 0;
			}
		}
	}
}

void temperaturaAmbiente(){
	leTemperatura(MLX90614_TA);
	informacoesPainel[5] = bufTemp[0];
	informacoesPainel[6] = bufTemp[1];
}

void temperaturaObjeto(){
	leTemperatura(MLX90614_TOBJ);
	informacoesPainel[7] = bufTemp[0];
	informacoesPainel[8] = bufTemp[1];
}

void leTemperatura(uint8_t reg){
	HAL_I2C_Mem_Read(&hi2c2, (MLX90614_ADDR<<1), reg, 1, bufTemp, 3, 100);

}

void leituraGiroscopio(){
	leituraRegMPU(MPU6050_GIROSCOPIO);
	for(int i = 0; i < 6; i++){
		informacoesPainel[9+i] = bufMPU[i];
	}
}

void leituraAcelerometro(){
	leituraRegMPU(MPU6050_ACELEROMETRO);
	for(int i = 0; i < 6; i++){
		informacoesPainel[15+i] = bufMPU[i];
	}
}

void leituraRegMPU(uint8_t reg){
	HAL_I2C_Mem_Read(&hi2c2, (MPU6050_ADDR<<1), reg, 1, bufMPU, 6, 100);
}

/* USER CODE END 4 */

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
