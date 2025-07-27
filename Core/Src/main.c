/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "encoder.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include "jy61p.h"
#include "pid_control.h"
#include "angle_control.h"
#include "flow_decode.h"

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

/* USER CODE BEGIN PV */

/* The variables about GRYO is declared here */

volatile uint8_t gyro_sample = 0; // How many seconds to read data
uint8_t g_usart5_receivedata;

/* This is the end of declaration */

/* This is the declaration of the variables of encoder data*/

extern Encoder_TypeDef encoderA;
extern Encoder_TypeDef encoderB;
volatile uint8_t encoder_sample = 0; // How many seconds to read encoder data

/* This is the end of declaration to the encoder*/

/* This is for the time of user_led */
volatile uint8_t user_led_time = 0; // How many seconds to turn on the user led
/* This is the end */

/* This is the declaration of the variables of angle PID control*/
volatile uint8_t angle_pid_sample = 0; // How many ms to update angle PID
/* This is the end of declaration to the angle PID*/

/* This is the space for you to place optical_flow */
volatile uint8_t optical_flow_sample = 0; // How many seconds to read data from optical flow sensor
unsigned char ch;
/* This is the end of optical_flow declaration */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Init uart receive
  UART_StartReceive(&huart4);
  HAL_UART_Receive_IT(&huart5, &g_usart5_receivedata, 1);
  HAL_TIM_Base_Start_IT(&htim5);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  Motor_Init();
  Encoder_Init();

  /* Init the PID control here */
  PID_Init(&pid_motor_a);
  PID_Init(&pid_motor_b);
  AngleControl_Init();
  SetStraightDrive(80.0f, 0.0f);
  /* Finish init PID control */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_UART_Receive(&huart2, &ch, 1, 25);
    /* This is for our led to check */
    if (user_led_time >= 99)
    {
      user_led_time = 0;                                    // Reset the time
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin); // Toggle the user led
    }
    /* This is the end of check, don't move! */

    /* This is for the encoder to update */
    if (encoder_sample >= 49)
    {
      AnglePID_Update();
      Encoder_Update(&encoderA, 50); // Put into the sample time(ms)
      Encoder_Update(&encoderB, 50);
      PID_Update();
      encoder_sample = 0; // Reset the sample counter
      // printf("Yaw: %.2f, rppA: %.2f, rppB: %.2f\r\n",
      //        My_YawAngle,
      //        encoderA.speed_rpp,
      //        encoderB.speed_rpp);
    }

    /* This is the code for angle update */

    /* This is the call to the function of read the data of optical flow sensor */
    if (optical_flow_sample >= 99)
    {
      static int ret;
      ret = upnotof_parse_char(ch);
      if (!ret)
      {
        static int16_t flow_x_integral = 0;
        static int16_t flow_y_integral = 0;
        static uint8_t valid = 0;
        flow_x_integral = up_flow_data.flow_x_integral;
        flow_y_integral = up_flow_data.flow_y_integral;
        valid = up_flow_data.valid;
        printf("flow_x_integral=%d,flow_y_integral=%d,valid=%d\n",
               flow_x_integral, flow_y_integral, valid);
      }
      optical_flow_sample = 0; // Reset the sample counter
    }
    /* This is the end of this function */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Timer interruption is created here. Don't delete it!!! */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // All Timed Tasks Need to be marked here!!!
  if (htim == &htim5)
  {
    if (user_led_time < 99)
    {
      user_led_time++;
    }

    if (encoder_sample < 69)
    {
      encoder_sample++;
    }
    if (angle_pid_sample <= 29)
    {
      angle_pid_sample++;
    }
    if (optical_flow_sample <= 99)
    {
      optical_flow_sample++;
    }
    /* End of temp */
  }
}
/* Don't Move This Code!!! */

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
