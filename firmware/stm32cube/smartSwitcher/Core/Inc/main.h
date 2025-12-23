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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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

void LightControlDM1();
void LightControlDM2();

void sendTempToCAN();

void INT_SW1();
void INT_SW2();
void INT_SW3();
void lightSet(uint8_t ch, uint8_t light, uint8_t time, uint8_t startValue);
void lightEn(uint8_t ch, uint8_t en);
long map(long x, long in_min, long in_max, long out_min, long out_max);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOA
#define SINC_Pin GPIO_PIN_3
#define SINC_GPIO_Port GPIOA
#define TEMP_Pin GPIO_PIN_5
#define TEMP_GPIO_Port GPIOA
#define DIM1_Pin GPIO_PIN_8
#define DIM1_GPIO_Port GPIOB
#define DIM2_Pin GPIO_PIN_9
#define DIM2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MAX_DIM 0
#define MIN_DIM 200
#define CH_DIM_1 0
#define CH_DIM_2 1

typedef struct Board {
    uint8_t Adres; // Адрес платы
    
} board_t;
extern board_t Brd;

typedef struct Telemetry {
    uint8_t sw1; // Состояние кнопки SW1
    uint8_t sw2; // Состояние кнопки SW2  
    uint8_t sw3; // Состояние кнопки SW3
    int16_t temp; // Температура в сотых долях градуса
  } telemetry_t;
extern telemetry_t BrdTel;

typedef struct Dimmer{
    uint8_t dimmer; // Значение яркости
    uint8_t timeOn; // Время изменения включения
    uint8_t timeOff; // Время изменения выключения
    uint8_t startValue; // Начальное значение для плавного изменения
    uint8_t brightness;
} dimmer_t;
extern dimmer_t dm[2];


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
