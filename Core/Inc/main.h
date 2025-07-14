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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
extern volatile int32_t encoder_count_prev_E2;

typedef struct speed_data{
  float speed;
  float distance;//
  int32_t encoder_count_prev;
  int32_t current_count;
  float delta_pulses;
  float delta_distance;
} Speed_Data;
#define wheel 6.5f
#define wheel_encoder 6.0f
extern char imu_rx_buffer[4096]__attribute__((section(".out")));
extern char speed_rx_buffer[4096]__attribute__((section(".out")));
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN1_Pin GPIO_PIN_13
#define AIN1_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_0
#define BIN1_GPIO_Port GPIOF
#define CIN2_Pin GPIO_PIN_1
#define CIN2_GPIO_Port GPIOF
#define CIN1_Pin GPIO_PIN_2
#define CIN1_GPIO_Port GPIOF
#define DIN2_Pin GPIO_PIN_3
#define DIN2_GPIO_Port GPIOF
#define DIN1_Pin GPIO_PIN_4
#define DIN1_GPIO_Port GPIOF
#define AIN2_Pin GPIO_PIN_6
#define AIN2_GPIO_Port GPIOF
#define BIN2_Pin GPIO_PIN_7
#define BIN2_GPIO_Port GPIOF
#define SPI_CS_Pin GPIO_PIN_3
#define SPI_CS_GPIO_Port GPIOC
#define END1_Pin GPIO_PIN_0
#define END1_GPIO_Port GPIOA
#define END2_Pin GPIO_PIN_1
#define END2_GPIO_Port GPIOA
#define ENA1_Pin GPIO_PIN_5
#define ENA1_GPIO_Port GPIOA
#define ENB1_Pin GPIO_PIN_6
#define ENB1_GPIO_Port GPIOA
#define ENB2_Pin GPIO_PIN_7
#define ENB2_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_9
#define PWMA_GPIO_Port GPIOE
#define PWMB_Pin GPIO_PIN_11
#define PWMB_GPIO_Port GPIOE
#define PWMC_Pin GPIO_PIN_13
#define PWMC_GPIO_Port GPIOE
#define PWMD_Pin GPIO_PIN_14
#define PWMD_GPIO_Port GPIOE
#define ENC2_Pin GPIO_PIN_12
#define ENC2_GPIO_Port GPIOD
#define ENC1_Pin GPIO_PIN_13
#define ENC1_GPIO_Port GPIOD
#define ENA2_Pin GPIO_PIN_3
#define ENA2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
