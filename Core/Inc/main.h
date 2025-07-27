/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern uint8_t g_usart5_receivedata;
extern unsigned char ch;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOC
#define General_SDA_Pin GPIO_PIN_0
#define General_SDA_GPIO_Port GPIOF
#define General_SCL_Pin GPIO_PIN_1
#define General_SCL_GPIO_Port GPIOF
#define MotorPWM_A_Pin GPIO_PIN_5
#define MotorPWM_A_GPIO_Port GPIOA
#define EncoderB_1_Pin GPIO_PIN_6
#define EncoderB_1_GPIO_Port GPIOA
#define EncoderB_2_Pin GPIO_PIN_7
#define EncoderB_2_GPIO_Port GPIOA
#define Key1_Pin GPIO_PIN_2
#define Key1_GPIO_Port GPIOB
#define Btn1_Pin GPIO_PIN_0
#define Btn1_GPIO_Port GPIOG
#define Btn2_Pin GPIO_PIN_1
#define Btn2_GPIO_Port GPIOG
#define MotorA_control_2_Pin GPIO_PIN_8
#define MotorA_control_2_GPIO_Port GPIOE
#define EncoderA_1_Pin GPIO_PIN_9
#define EncoderA_1_GPIO_Port GPIOE
#define MotorA_control_1_Pin GPIO_PIN_10
#define MotorA_control_1_GPIO_Port GPIOE
#define EncoderA_2_Pin GPIO_PIN_11
#define EncoderA_2_GPIO_Port GPIOE
#define PWM_1_Pin GPIO_PIN_10
#define PWM_1_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_11
#define PWM_2_GPIO_Port GPIOB
#define ServoPWM_3_Pin GPIO_PIN_12
#define ServoPWM_3_GPIO_Port GPIOD
#define ServoPWM_4_Pin GPIO_PIN_13
#define ServoPWM_4_GPIO_Port GPIOD
#define PWM_5_Pin GPIO_PIN_14
#define PWM_5_GPIO_Port GPIOD
#define PWM_6_Pin GPIO_PIN_15
#define PWM_6_GPIO_Port GPIOD
#define Red_line_1_Pin GPIO_PIN_2
#define Red_line_1_GPIO_Port GPIOG
#define Red_line_2_Pin GPIO_PIN_3
#define Red_line_2_GPIO_Port GPIOG
#define Red_line_3_Pin GPIO_PIN_4
#define Red_line_3_GPIO_Port GPIOG
#define Red_line_4_Pin GPIO_PIN_5
#define Red_line_4_GPIO_Port GPIOG
#define USART_TX_TTL_Pin GPIO_PIN_9
#define USART_TX_TTL_GPIO_Port GPIOA
#define USART_RX_TTL_Pin GPIO_PIN_10
#define USART_RX_TTL_GPIO_Port GPIOA
#define UART_TX_BLUETOOTH_Pin GPIO_PIN_10
#define UART_TX_BLUETOOTH_GPIO_Port GPIOC
#define UART_RX_BLUETOOTH_Pin GPIO_PIN_11
#define UART_RX_BLUETOOTH_GPIO_Port GPIOC
#define URAT_TX_GYRO_Pin GPIO_PIN_12
#define URAT_TX_GYRO_GPIO_Port GPIOC
#define MotorB_control_1_Pin GPIO_PIN_0
#define MotorB_control_1_GPIO_Port GPIOD
#define MotorB_control_2_Pin GPIO_PIN_1
#define MotorB_control_2_GPIO_Port GPIOD
#define URAT_RX_GYRO_Pin GPIO_PIN_2
#define URAT_RX_GYRO_GPIO_Port GPIOD
#define Key8_Pin GPIO_PIN_3
#define Key8_GPIO_Port GPIOD
#define Beep_Pin GPIO_PIN_4
#define Beep_GPIO_Port GPIOD
#define Flow_TX_Pin GPIO_PIN_5
#define Flow_TX_GPIO_Port GPIOD
#define Flow_RX_Pin GPIO_PIN_6
#define Flow_RX_GPIO_Port GPIOD
#define Key2_Pin GPIO_PIN_7
#define Key2_GPIO_Port GPIOD
#define Key3_Pin GPIO_PIN_9
#define Key3_GPIO_Port GPIOG
#define Key4_Pin GPIO_PIN_10
#define Key4_GPIO_Port GPIOG
#define Key5_Pin GPIO_PIN_12
#define Key5_GPIO_Port GPIOG
#define Key6_Pin GPIO_PIN_13
#define Key6_GPIO_Port GPIOG
#define Key7_Pin GPIO_PIN_14
#define Key7_GPIO_Port GPIOG
#define MotorPWM_B_Pin GPIO_PIN_3
#define MotorPWM_B_GPIO_Port GPIOB
#define Oled_SCL_Pin GPIO_PIN_6
#define Oled_SCL_GPIO_Port GPIOB
#define Oled_SDA_Pin GPIO_PIN_7
#define Oled_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
