/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_adc.h"
#include <stdlib.h> // Для функции abs
#include <stdint.h>

adcval_t Adc = {0};  // Инициализация структуры значений ADC
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA5     ------> ADC1_IN5
    */
    GPIO_InitStruct.Pin = TEMP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(TEMP_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA5     ------> ADC1_IN5
    */
    HAL_GPIO_DeInit(TEMP_GPIO_Port, TEMP_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1) /* Check if the interrupt comes from ACD1 */
	    {
	    	/* Set flag to true */
				Adc.FadcChanal = FiltrRC1(Adc.adcChanal[CHANAL_ADC_TEMP]);
       // BrdTel.temp = (int16_t)(readTemperatureLM335(Adc.FadcChanal)*10);
      }

}

double readTemperatureLM335(uint16_t data) {
  uint32_t vdda = 0;

  /*
   * Some STM32 families (F0) provide helper macro __LL_ADC_CALC_VREFANALOG_VOLTAGE
   * that computes VDDA (in mV) from the VREFINT ADC measurement and factory calibration.
   * On other families (F1) this macro is not present. Provide guarded handling:
   *  - if macro is available, use it;
   *  - else if VREFINT calibration symbols exist, use them;
   *  - otherwise fall back to a default VDDA value (3300 mV).
   */
#if defined(__LL_ADC_CALC_VREFANALOG_VOLTAGE)
  vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(Adc.adcChanal[CHANAL_ADC_VREF], LL_ADC_RESOLUTION_12B);
#elif defined(VREFINT_CAL_ADDR) && defined(VREFINT_CAL_VREF)
  /* Protect against division by zero */
  if (Adc.adcChanal[CHANAL_ADC_VREF] != 0)
  {
    /* VREFINT_CAL_ADDR holds factory raw ADC value for VREFINT at Vref = VREFINT_CAL_VREF */
    vdda = (uint32_t)(((uint64_t)(*VREFINT_CAL_ADDR) * (uint64_t)VREFINT_CAL_VREF) / (uint64_t)Adc.adcChanal[CHANAL_ADC_VREF]);
  }
  else
  {
    vdda = 3300U; /* fallback */
  }
#else
  /*
   * STM32F1: no VREFINT calibration value in system memory on many devices.
   * Use typical VREFINT voltage (approx. 1.20 V) from datasheet to estimate VDDA:
   *   VDDA = VREFINT_TYP_mV * ADC_FULL_SCALE / VREFINT_ADC_RAW
   * This gives a best-effort VDDA; accuracy depends on VREFINT tolerance.
   */
  const uint32_t VREFINT_TYP_MV = 1170U; /* typical VrefInt voltage in mV for many STM32F1 devices */
  uint32_t vref_raw = Adc.adcChanal[CHANAL_ADC_VREF];
  if (vref_raw != 0U) {
    vdda = (uint32_t)((uint64_t)VREFINT_TYP_MV * 4095ULL / (uint64_t)vref_raw);
  } else {
    vdda = 3300U; /* fallback if measurement is zero */
  }
#endif

    /* Convert ADC temperature reading to millivolts using VDDA (mV) */
    double voltage_mV = ((double)data) * ((double)vdda) / 4095.0;
    /* LM335: 10 mV per Kelvin */
    double kelvin = voltage_mV / 10.0;
    double temperatureC = kelvin - 273.15;
    return temperatureC;
}

uint16_t FiltrRC1(uint16_t data)
{
    static uint32_t Dacc = 0;
    static uint16_t Dout = 0;
    uint16_t Din = data;

    Dacc = Dacc + Din - Dout;
    Dout = Dacc / (uint16_t)KOF_RC_FILTR;

    return Dout;
}
/* USER CODE END 1 */
