/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LED_ON() HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1)
#define LED_OFF() HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0)
#define LED_TOGGLE() HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char TX_buffer[64];

uint8_t start=0,
				teste_pwm,
				pwm_final,
				RAD_S_INT,
				ADC_FILTER_SWITCH = 1;
				
int16_t error_int;
			
uint16_t STEP = 0,
				 subcount = 0,
				 RefreshTimerAquisition = 1000,
				 MotorVoltageInt;

uint32_t ADC_RAW[2],
				 encoderCount,
				 Last_encoderCount,
				 counter = 0,
				 SPEED_TIMER = 0;

float RPS = 0,
			RAD_S,
			RAD_S_RAW,
			RAD_S_1,
			rotacoes = 0,
			error,
			error_1,
			setpoint = 0, 
      TsEncoder,
      MotorVoltage;
			
uint8_t Rx_data[10];

float vref = 3.3f,
			divisor = 4.2f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void itoa(unsigned long val, char* asc);
void uint_to_asc(unsigned long val1, unsigned long val2, unsigned long val3, unsigned long val4,
                 unsigned long val5 ,char *buffer);

void H_dir(uint8_t dir);
void pwm(uint16_t duty);
void sendchar(uint8_t c);
void sendstring(char *string);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_Delay(100);
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim1);
	//HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, ADC_RAW, 2); 
	
	HAL_UART_Receive_IT (&huart2, Rx_data, 4);
	
	H_dir(1);
	
	pwm(0);
	STEP = 0;	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0)
		{
		}
		
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0)	
		{
		}
		
		if(__HAL_TIM_GET_COUNTER(&htim1) > 5000 - 1)  //freq de controle
		{
			//LED_ON();
			__HAL_TIM_SET_COUNTER(&htim1, 0);
			TsEncoder = RefreshTimerAquisition / 1000000.0f; //calculo do Ts
			
			encoderCount = __HAL_TIM_GET_COUNTER(&htim2); //le o encoder
			RPS = (encoderCount - Last_encoderCount) / 20.0f; //calcula RPS
			RAD_S = RPS * 6.283185f; //transforma em rad/s
						
			Last_encoderCount = encoderCount; //guarda a leitura z-1
			rotacoes += RPS * TsEncoder; //aculuma as rotacoes					
			
			pwm_final = teste_pwm * 10;
			pwm(pwm_final); //insere a acao de controle na saida	
			
			RAD_S_INT = RAD_S * 10;
			MotorVoltageInt = MotorVoltage * 10;
						
			sprintf(TX_buffer, "%u,%u,%u,%u,%u\r\n", counter, RAD_S_INT, pwm_final, MotorVoltageInt, ADC_RAW[1]);
			HAL_UART_Transmit(&huart2, (uint8_t*)TX_buffer, strlen(TX_buffer),4);
			
			//LED_OFF();
		}
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	
	LED_TOGGLE();
	
//	ADC_RAW[0] = HAL_ADC_GetValue(&hadc1);
//	ADC_RAW[1] = HAL_ADC_GetValue(&hadc1);

	if (pwm_final < 1)
			ADC_RAW[0] = 0;

	//MotorVoltage = ((vref/4095) * ADC_RAW) * divisor;
	
	HAL_ADC_Start_DMA(&hadc1, ADC_RAW, 2); 
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//pwm(STEP);
}

void uint_to_asc(unsigned long val1, unsigned long val2, unsigned long val3, unsigned long val4,
                 unsigned long val5, char *buffer)
{
    char bff_aux[10];
    int i = 0;
    
    itoa(val1, bff_aux);
    for(i=0; i<6;i++)
    {
        buffer[i] = bff_aux[i];
    }
    buffer[6] = ',';
    
    itoa(val2, bff_aux);
    for(i=7; i<13;i++)
    {
        buffer[i] = bff_aux[i-7];
    }
    buffer[13] = ',';
    
    itoa(val3, bff_aux);
    for(i=14; i<20;i++)
    {
        buffer[i] = bff_aux[i-14];
    }
    buffer[20] = ',';
    
    itoa(val4, bff_aux);
    for(i=21; i<27;i++)
    {
        buffer[i] = bff_aux[i-21];
    }
    buffer[27] = ',';
    
    itoa(val5, bff_aux);
    for(i=28; i<34;i++)
    {
        buffer[i] = bff_aux[i-28];
    }
    buffer[34] = '\r';
    buffer[35] = '\n';
}


void itoa(unsigned long val, char* asc)
{
    asc[5] = val%10 + '0';
    asc[4] = (val/10)%10 + '0';
    asc[3] = (val/100)%10 + '0';
    asc[2] = (val/1000)%10 + '0';
    asc[1] = (val/10000)%10 + '0';
    asc[0] = (val/100000)%10 + '0';
}


void H_dir(uint8_t dir)
{
	if(dir == 0)
	{
		HAL_GPIO_WritePin(IN1_H_GPIO_Port, IN1_H_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_H_GPIO_Port, IN2_H_Pin, GPIO_PIN_RESET);
	}
	
	if(dir == 1)
	{
		HAL_GPIO_WritePin(IN1_H_GPIO_Port, IN1_H_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_H_GPIO_Port, IN2_H_Pin, GPIO_PIN_SET);
	}
	
	if(dir == 2)
	{
		HAL_GPIO_WritePin(IN1_H_GPIO_Port, IN1_H_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_H_GPIO_Port, IN2_H_Pin, GPIO_PIN_RESET);
	}
}

void pwm(uint16_t duty)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);
}

void sendchar(uint8_t c)
{
	while(!(USART2->SR &(1<<6)));
	USART2->DR = c;
	
}

void sendstring(char *string)
{
	while(*string) sendchar(*string++);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if(Rx_data[0] == 's')
	{
		setpoint = ((Rx_data[1] - '0') * 100) + ((Rx_data[2] - '0') * 10) + (Rx_data[3] - '0');
	}
  HAL_UART_Receive_IT(&huart2, Rx_data, 4); 
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
