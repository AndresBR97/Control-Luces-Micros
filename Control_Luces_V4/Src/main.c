/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define REF_Debounce 300
#define REF_Debounce_2 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//Para modos de funcionamiento
volatile int EnEspera_Funcionando = 0;
volatile int user_mode = 0;

//Para el debounce 
extern uint32_t last_time;
extern uint32_t counter_ms;

//Para estado de los pulsadores
int boton_pulsado_modo = 0;
int boton_pulsado_azul = 0;
int boton_pulsado_L1 = 0;
int boton_pulsado_L2 = 0;
int boton_pulsado_L3 = 0;

//Para marcar en que momento de la secuencia de user_mode = 1 se encuentra
volatile int paso = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
void ConfigureSysTick(void);
uint32_t millis(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	
	//Inicialización de timers
	SysTick_Config(SystemCoreClock/1000); //Para interrupciones cada 1ms
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	NVIC_DisableIRQ(TIM3_IRQn);				
		
	//Inicialización de variables
	user_mode = 0;
	EnEspera_Funcionando = 0;
	boton_pulsado_modo = 0;
	boton_pulsado_azul = 0;
  boton_pulsado_L1 = 0;
  boton_pulsado_L2 = 0;
  boton_pulsado_L3 = 0;
	paso = 0;	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && boton_pulsado_azul == 0){
			last_time = millis(); //Guarda el valor de millis()
			boton_pulsado_azul = 1;			
		}
		
		if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) && (millis() - last_time) > REF_Debounce && (boton_pulsado_azul == 1)){
			
			boton_pulsado_azul = 0;
			
			if (EnEspera_Funcionando == 0){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);		
				EnEspera_Funcionando = 1;
			}	

			else if(EnEspera_Funcionando == 1){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);		
				EnEspera_Funcionando = 0;
			}
		}		
		
		if (EnEspera_Funcionando == 0){
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); //Led rojo se apaga	
			
			//Se pulsa el boton de cambio de modo
			if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) && boton_pulsado_modo == 0){							
					last_time = millis(); //Guarda el valor de millis()
					boton_pulsado_modo = 1;
			}
			
			//Se confirma que se ha pulsado y cambia a modo automático
			if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) && (millis() - last_time) > REF_Debounce_2 && (boton_pulsado_L1 == 1) && user_mode == 0){
					NVIC_EnableIRQ(TIM3_IRQn);
					HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Led verde encendido									
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
					user_mode = 1;		
			}
			
			//Se confirma que se ha pulsado y cambia a modo manual	
			else if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) && (millis() - last_time) > REF_Debounce_2 && (boton_pulsado_L1 == 1) && user_mode == 1){
					NVIC_DisableIRQ(TIM3_IRQn);				
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);						
					user_mode = 0;		
			}		
			
			//Usuario en modo manual
			if (user_mode == 0){
				
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //Led verde apagado
				
				//Si se pulsa el primer pulsador...
				if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == 0 && boton_pulsado_L1 == 0){
					last_time = millis(); //Guarda el valor de millis()
					boton_pulsado_L1 = 1;
				}
				
				if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == 0) && (millis() - last_time) > REF_Debounce && (boton_pulsado_L1 == 1)){
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
						boton_pulsado_L1 = 0;
				}		
				
				//Si se pulsa el segundo pulsador...
				if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == 0 && boton_pulsado_L2 == 0){
					last_time = millis(); //Guarda el valor de millis()
					boton_pulsado_L2 = 1;
				}
				
				if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == 0) && (millis() - last_time) > REF_Debounce && (boton_pulsado_L1 == 1)){
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
						boton_pulsado_L2 = 0;
				}	
				
				//Si se pulsa el tercer pulsador...
				if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0 && boton_pulsado_L3 == 0){
					last_time = millis(); //Guarda el valor de millis()
					boton_pulsado_L3 = 1;
				}
				
				if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) && (millis() - last_time) > REF_Debounce && (boton_pulsado_L3 == 1)){
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
						boton_pulsado_L3 = 0;
				}							
			}	
		}
		
		else if (EnEspera_Funcionando == 1){ //Modo standby
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); //Led rojo
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //Led verde
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
			HAL_Delay(200);
		}
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
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
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2){	//Si la fuente es TIM2	

	}
	
	else if (htim->Instance == TIM3){	//Si la fuente es TIM3
		
		if (user_mode == 1){
			
			if (paso == 0){
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
				paso++;
			}
			
			else if (paso == 1){
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
				paso++;
			}
			
			else if (paso == 2){
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
				paso++;
			}
			
			else if (paso == 3){
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
				paso++;
			}
			
			else if (paso == 4){
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
				paso = 0;
			}
		}
	}
}

uint32_t millis() {
  return counter_ms;
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
