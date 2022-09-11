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
//#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "filter_1order.h"
#include "Map.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_BUF_SIZE 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char TX_buffer[64];

uint32_t teste_pwm,
				pwm_final;

uint32_t ADC_RAW[ADC_BUF_SIZE],
				 counter = 0;
			
uint8_t Rx_data[10];

uint32_t SPEED_TIMER,
AuxTimerMs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void itoa(unsigned long val, char* asc);
void x5uint_to_asc(unsigned long val1, unsigned long val2, unsigned long val3, unsigned long val4,
                 unsigned long val5 ,char *buffer);
void x3uint_to_asc(unsigned long val1, unsigned long val2, unsigned long val3, char *buffer);

void H_dir(uint8_t dir);
void pwm(uint16_t duty);
void sendchar(uint8_t c);
void sendstring(char *string);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t CurrentIndex;
volatile uint32_t AvgAdcCurrent, SumAdcCurrent, Print, Sps;

volatile uint32_t Freq = 1000-1;

uint32_t teste;
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
  //MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	
	H_dir(1);
	HAL_Delay(100);
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim1);
		
	HAL_UART_Receive_IT (&huart2, Rx_data, 4);
		
	htim3.Init.Prescaler = 72-1;
	htim3.Init.Period = 1000-1;
	HAL_TIM_Base_Init(&htim3); 
	HAL_TIM_Base_Start(&htim3);
	
	HAL_ADC_Start_DMA(&hadc1, ADC_RAW, ADC_BUF_SIZE); 
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	
	pwm(0);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {				
		if(GetTimer1() > Freq)
		{
			ResetTimer1();
			LED2_ON();
						
			pwm_final = teste_pwm * 10;
			pwm(pwm_final); //insere a acao de controle na saida	
			
			//MotorVoltageInt = MotorVoltage * 10;	
			
			for(uint32_t j=0; j<ADC_BUF_SIZE; j++)
			{
				SumAdcCurrent += ADC_RAW[j];
			}
			AvgAdcCurrent = SumAdcCurrent / ADC_BUF_SIZE;
			
			Sps = CurrentIndex * 10000 * ADC_BUF_SIZE; /* 2000 is 2Khz (500uS) */
			
			CurrentIndex = 0;
			SumAdcCurrent = 0;
			
			//sprintf(TX_buffer, "%u,%u,%u\r\n", counter, pwm_final, AvgAdcCurrent);
			x3uint_to_asc(counter, pwm_final, AvgAdcCurrent, TX_buffer);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)TX_buffer, 22);
			
			LED2_OFF();
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	
	LED1_TOGGLE();
	CurrentIndex++;
	HAL_ADC_Start_DMA(&hadc1, ADC_RAW, ADC_BUF_SIZE); 
}

void x5uint_to_asc(unsigned long val1, unsigned long val2, unsigned long val3, unsigned long val4,
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

void x3uint_to_asc(unsigned long val1, unsigned long val2, unsigned long val3, char *buffer)
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

    buffer[20] = '\r';
    buffer[21] = '\n';
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
		//setpoint = ((Rx_data[1] - '0') * 100) + ((Rx_data[2] - '0') * 10) + (Rx_data[3] - '0');
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
