/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
enum{
	CHANALS_ADC = 3,
	CHANAL_ADC_TEMP =0,
	CHANAL_ADC_TMPCHIP =1,
	CHANAL_ADC_VREF =2,
	ADC_MAX_VAL=4094
};

#define KOF_RC_FILTR 10
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct AdcValues{
	uint16_t adcChanal[CHANALS_ADC]; //0-3 pin 4 temp 5 vref
	uint16_t FadcChanal;
	uint16_t TEMP;
}adcval_t;
extern adcval_t Adc;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
uint16_t FiltrRC1(uint16_t data);
double readTemperatureLM335(uint16_t data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

