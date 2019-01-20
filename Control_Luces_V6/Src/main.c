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
#include <string.h>
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REF_Debounce 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
//Para modos de funcionamiento
volatile int EnEspera_Funcionando = 0;
volatile int user_mode = 0;
int Sis_BT = 0;

//Para el debounce 
extern uint32_t last_time;
extern uint32_t counter_ms;

//Para estado de los pulsadores
int boton_pulsado_modo = 0;
int boton_pulsado_azul = 0;
int boton_pulsado_L1 = 0;
int boton_pulsado_L2 = 0;
int boton_pulsado_L3 = 0;
int N = 0;

//Para marcar en que momento de la secuencia de user_mode = 1 se encuentra
volatile int paso = 0;

//Para el conversor analógico-digital
uint32_t adcValue;
float temperature;
float temp_ambiente;
int temperatura_tomada;
int sensor_activa;

//Para comunicación serie bluetooth
char rx_buffer[10];
char tx_buffer[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	
	//Inicialización de timers
	SysTick_Config(SystemCoreClock/1000); //Para interrupciones cada 1ms
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
	
	//Para el oonversor analógico-digital
	adcValue = 0;
	temperature = 0;
	temp_ambiente = 0;
	temperatura_tomada = 0;
	sensor_activa = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
////////////////////////////////////////////CÓDIGO PARA EL FUNCIONAMIENTO DEL SENSOR DE TEMPERATURA////////////////////////////////////////////
		
		HAL_ADC_Start(&hadc1);
		
		if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){ 
			adcValue = HAL_ADC_GetValue(&hadc1); //Si la conversión es exitosa, leemos el valor adc.
		}
		
		if(adcValue > 0){
			temperature = (float)adcValue/1024; 
			temperature = temperature * 0.7; //Obtener temperatura en forma de voltaje
			temperature = temperature - 0.01;	//Resta el offset de la recta
			temperature = temperature * 100; //Obtener temperatura en forma de grados
		}
		
		if (temperatura_tomada == 0){
			temp_ambiente = temperature;		
			int int_temp_ambiente = (int)temp_ambiente;		
			
			HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Temperatura ambiente: %d\n", int_temp_ambiente), 500);			
			temperatura_tomada = 1;
		}
		
		if (temperature >= (temp_ambiente + 5)){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		}
		
		else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);		
		}
		
		HAL_Delay(50);
		
////////////////////////////////////////////CÓDIGO PARA EL FUNCIONAMIENTO DEL SISTEMA DE LUCES CON MOVIL////////////////////////////////////////////		
	 
		HAL_UART_Receive(&huart6, (uint8_t*)rx_buffer, 10, 500);
			
			//MOSTRAR TEMPERTAURA
			if(rx_buffer[0] == 'T' && rx_buffer[1] == 'E' && rx_buffer[2] == 'M' && rx_buffer[3] == 'P'){
				int int_temperatura = temperature;
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Temperatura: %d\n", int_temperatura), 500);		
				memset (rx_buffer, 0, 10);				
			}
			
			//LUZ 1
			else if(rx_buffer[0] == 'L' && rx_buffer[1] == '1' && rx_buffer[2] == 'O' && rx_buffer[3] == 'N'){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
				memset (rx_buffer, 0, 10);
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Lampara 1 encendida\n"), 500);
			}
			
			else if(rx_buffer[0] == 'L' && rx_buffer[1] == '1' && rx_buffer[2] == 'O' && rx_buffer[3] == 'F' && rx_buffer[4] == 'F'){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
				memset (rx_buffer, 0, 10);
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Lampara 1 apagada\n"), 500);			
			}
			
			//LUZ 2
			else if(rx_buffer[0] == 'L' && rx_buffer[1] == '2' && rx_buffer[2] == 'O' && rx_buffer[3] == 'N'){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
				memset (rx_buffer, 0, 10);				
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Lampara 2 encendida\n"), 500);				
			}
			
			else if(rx_buffer[0] == 'L' && rx_buffer[1] == '2' && rx_buffer[2] == 'O' && rx_buffer[3] == 'F' && rx_buffer[4] == 'F'){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				memset (rx_buffer, 0, 10);
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Lampara 2 apagada\n"), 500);			
			
			}
			
			//LUZ 3
			else if(rx_buffer[0] == 'L' && rx_buffer[1] == '3' && rx_buffer[2] == 'O' && rx_buffer[3] == 'N'){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
				memset (rx_buffer, 0, 10);	
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Lampara 3 encendida\n"), 500);								
			}
			
			else if(rx_buffer[0] == 'L' && rx_buffer[1] == '3' && rx_buffer[2] == 'O' && rx_buffer[3] == 'F' && rx_buffer[4] == 'F'){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
				memset (rx_buffer, 0, 10);
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Lampara 3 apagada\n"), 500);								
			
			}
			
////////////////////////////////////////CÓDIGO PARA EL FUNCIONAMIENTO DEL SISTEMA DE LUCES SIN MÓVIL////////////////////////////////////////
		
		if (rx_buffer[0] == 'S' && rx_buffer[1] == 'I' && rx_buffer[2] == 'S' && rx_buffer[3] == 'O' && rx_buffer[4] == 'F' && rx_buffer[5] == 'F'){
			Sis_BT = 1; //EL SISTEMA PASA A ESTADO STANDBY POR BLUETOOTH
		}
		
		else if (rx_buffer[0] == 'S' && rx_buffer[1] == 'I' && rx_buffer[2] == 'S' && rx_buffer[3] == 'O' && rx_buffer[4] == 'N') {
			Sis_BT = 2; //EL SISTEMA PASA A ESTADO DE FUNCIONAMIENTO POR BLUETOOTH			
		}
			
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && boton_pulsado_azul == 0){
			last_time = millis(); //Guarda el valor de millis()
			boton_pulsado_azul = 1;			
		}
		
		if (((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) && ((millis() - last_time) > REF_Debounce)) || Sis_BT == 1 || Sis_BT == 2){		
			boton_pulsado_azul = 0;
			
			if (EnEspera_Funcionando == 0 || Sis_BT == 1){ //APAGA EL SISTEMA (STANDBY)
				memset (rx_buffer, 0, 10);				
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);		
				EnEspera_Funcionando = 1;
				user_mode = 0;				
				Sis_BT = 0;
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Sistema apagado (stanby)\n"), 500);								
				
			}	

			else if(EnEspera_Funcionando == 1 || Sis_BT == 2){ //PONE EL SISTEMA EN FUNCIONAMIENTO
				memset (rx_buffer, 0, 10);				
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);		
				EnEspera_Funcionando = 0;
				user_mode = 0;				
				Sis_BT = 0;
				HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Sistema en funcionamiento\n"), 500);												
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
			if(((user_mode == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) && ((millis() - last_time) > REF_Debounce) && (boton_pulsado_modo == 1)) || (rx_buffer[0] == 'M' && rx_buffer[1] == 'O' && rx_buffer[2] == 'D' && rx_buffer[3] == 'A')){
					memset (rx_buffer, 0, 10);
					NVIC_EnableIRQ(TIM3_IRQn);
					HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Led verde encendido									
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
					user_mode = 1;
					boton_pulsado_modo = 0;
		  		HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Cambio a modo automatico\n"), 500);														
			}
			
			//Se confirma que se ha pulsado y cambia a modo manual	
			else if((user_mode == 1 && (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0) && ((millis() - last_time) > REF_Debounce) && (boton_pulsado_modo == 1)) || (rx_buffer[0] == 'M' && rx_buffer[1] == 'O' && rx_buffer[2] == 'D' && rx_buffer[3] == 'M')){
					memset (rx_buffer, 0, 10);
					NVIC_DisableIRQ(TIM3_IRQn);				
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);						
					user_mode = 0;	
					boton_pulsado_modo = 0;
		  		HAL_UART_Transmit(&huart6, (uint8_t*)tx_buffer, sprintf (tx_buffer, "Cambio a modo manual\n"), 500);																	
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
				if (temperature <= (temp_ambiente + 5)){
					if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == 0 && boton_pulsado_L2 == 0){
						last_time = millis(); //Guarda el valor de millis()
						boton_pulsado_L2 = 1;
					}
					
					if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == 0) && (millis() - last_time) > REF_Debounce && (boton_pulsado_L2 == 1)){
							HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
							boton_pulsado_L2 = 0;
					}
				}
				
				if (sensor_activa == 0 && temperature > (temp_ambiente + 5)){
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
					sensor_activa = 1;
				}
				
				else if (sensor_activa == 1 && (temperature <= (temp_ambiente + 5))){
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
					sensor_activa = 0;					
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
			
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
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
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
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
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
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
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3){	//Si la fuente es TIM3
		
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
