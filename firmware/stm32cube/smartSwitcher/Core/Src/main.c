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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_adc.h"
#include "flash_storage.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t tic = 0;
const uint16_t dimPins[] = {DIM1_Pin, DIM2_Pin};
board_t Brd;
telemetry_t BrdTel;
dimmer_t dm[2] = {
    {MAX_DIM, 10, 10,MIN_DIM, MAX_DIM}, // Диммер 1
    {MAX_DIM, 10, 10,MIN_DIM, MAX_DIM}  // Диммер 2
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//-------------прерывания по изменения яркости -----------------------
void LightControlDM1() {
    if (dm[CH_DIM_1].dimmer < MIN_DIM) {
        dm[CH_DIM_1].dimmer ++;
        if (dm[CH_DIM_1].dimmer >= MIN_DIM) {
            dm[CH_DIM_1].dimmer = MIN_DIM;
            HAL_TIM_Base_Stop_IT(&htim2);
        }
    } else if (dm[CH_DIM_1].dimmer > MAX_DIM) {
        dm[CH_DIM_1].dimmer --;
        if (dm[CH_DIM_1].dimmer <= MAX_DIM) {
            dm[CH_DIM_1].dimmer = MAX_DIM;
            HAL_TIM_Base_Stop_IT(&htim2);
        }
    }
}

void LightControlDM2() {
    if (dm[CH_DIM_2].dimmer < MIN_DIM) {
        dm[CH_DIM_2].dimmer ++;
        if (dm[CH_DIM_2].dimmer >= MIN_DIM) {
            dm[CH_DIM_2].dimmer = MIN_DIM;
            HAL_TIM_Base_Stop_IT(&htim3);
        }
    } else if (dm[CH_DIM_2].dimmer > MAX_DIM) {
        dm[CH_DIM_2].dimmer --;
        if (dm[CH_DIM_2].dimmer <= MAX_DIM) {
            dm[CH_DIM_2].dimmer = MAX_DIM;
            HAL_TIM_Base_Stop_IT(&htim3);
        }
    }
}

//-------------прерывания по кнопкам----------------
void INT_SW1(){
    BrdTel.sw1 = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);  
}
void INT_SW2(){
    BrdTel.sw2 = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);  
}
void INT_SW3(){
    BrdTel.sw3 = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);  
}


//------------отправка температуры по CAN--------
void sendTempToCAN(){
  BrdTel.temp = (int16_t)(readTemperatureLM335(Adc.FadcChanal) * 10);
  uint8_t tempData[2];
  tempData[0] = (uint8_t)(BrdTel.temp & 0xFF); // Младший байт
  tempData[1] = (uint8_t)(BrdTel.temp >> 8); // Старший байт
 
  AddMessageToQueue(Sh_Id(CAN_ID_TEMP), tempData, 2, SEND_ONLY);
}
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Adc.adcChanal, CHANALS_ADC);
  HAL_CAN_Start(&hcan);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim1); // таймер на детектора нуля
  HAL_TIM_Base_Start_IT(&htim4); // таймер на 30 сек для отправки температуры

  //----Считываем с флеш памяти адрес платы если там нет значение или = 255 то адресс 0
  Flash_ReadData(&Brd, sizeof(Brd));
  if (Brd.Adres == 0xFF) {
      Brd.Adres = 0;
      Flash_WriteData(&Brd, sizeof(Brd));
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     ProcessCANQueue();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void lightEn(uint8_t ch, uint8_t en) {
  
   if (en == 0) { // выключить
      dm[ch].dimmer = MAX_DIM;
   } else if (en == 1) { // включить
      dm[ch].dimmer = MIN_DIM;
   } else if (en == 2) { // плавное увеличение
      //-------включение таймера для плавного увеличения
      if(ch == CH_DIM_1) {
         SetTimeTimer_Dim1(dm[ch].timeOn);
         HAL_TIM_Base_Start_IT(&htim2);
      }
      if(ch == CH_DIM_2){
         SetTimeTimer_Dim2(dm[ch].timeOn);
         HAL_TIM_Base_Start_IT(&htim3);
      } 
     
   } else if (en == 3) { // плавное уменьшение
      //-------включение таймера для плавного уменьшения
     if(ch == CH_DIM_1) {
         SetTimeTimer_Dim1(dm[ch].timeOff);
         HAL_TIM_Base_Start_IT(&htim2);
      }
      if(ch == CH_DIM_2){
         SetTimeTimer_Dim2(dm[ch].timeOff);
         HAL_TIM_Base_Start_IT(&htim3);
      } 
   }
}

void lightSet(uint8_t ch, uint8_t value, uint8_t time, uint8_t startValue){
   if (value < MIN_DIM) value = MIN_DIM;
   if (value > MAX_DIM) value = MAX_DIM;
   dm[ch].dimmer = value;
   dm[ch].timeOn = time;
   dm[ch].startValue = startValue;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
#ifdef USE_FULL_ASSERT
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
