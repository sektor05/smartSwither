/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "string.h"
#include "adc.h"
#include "flash_storage.h"
#include <stdint.h>
//--------------формат инкремента для ЦАП


CAN_HandleTypeDef hcan;
CAN_TxHeaderTypeDef pTxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t TxMailbox;
uint8_t TX_data[8];

uint16_t valuePoint;

volatile bool ackReceived = false;  // Флаг, указывающий на получение ACK
#define CAN_QUEUE_SIZE 10  // Размер буфера сообщений
CANMessage sendDataCan[CAN_QUEUE_SIZE];
/* Debug counters for RX */
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_last_rx_id = 0xFFFFFFFF;
volatile uint32_t can_error_code = 0;
/* Runtime status flags for diagnostics (no UART needed) */
volatile bool can_started = false;
volatile bool can_notify_status = false;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterConfig(0, CAN_ID_GET_TEMP, 0x7F0); // настройка фильтра
  /* For debugging: accept all IDs while troubleshooting reception */
 // CAN_AcceptAllFilter();
  //CAN_FilterConfig(1, CAN_ID_SET_DIM1, 0x7FF); // настройка фильтра
  // CAN_FilterConfig(2, Sh_Id(CAN_ID_SET_DIM2), 0x7FF); // настройка фильтра
  // CAN_FilterConfig(3, Sh_Id(CAN_ID_EEPROM), 0x7FF); // настройка фильтра
  // CAN_FilterConfig(4, Sh_Id(CAN_ID_OKrx), 0x7FF); // настройка фильтра
  // CAN_FilterConfig(5, Sh_Id(CAN_ID_EN_LIGHT), 0x7FF); // настройка фильтра

 
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Don't hard-fail here. Try to report error and continue — CAN may be absent on the bus. */
    can_started = false;
  } else {
    can_started = true;
  }

  /* Enable receive notifications and error notifications so we can recover from bus-off, etc. */
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK)
  {
    /* Activation failed — do not stop the whole device. Will be handled in HAL callbacks. */
    can_notify_status = false;
  } else {
    can_notify_status = true;
  }
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* CAN1 interrupt Init: enable FIFO0 IRQ so HAL_CAN_RxFifo0MsgPendingCallback is called */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* CAN1 interrupt Deinit */
  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/*
 * Quick helper to set an accept-all filter for debugging.
 * Call CAN_AcceptAllFilter() from init or from a debug command to allow all IDs.
 */
void CAN_AcceptAllFilter(void)
{
  /* Filter 0, accept all IDs: startID=0, mask=0 -> all IDs pass */
  CAN_FilterConfig(0, 0, 0);
}

/* USER CODE BEGIN 1 */

bool AddMessageToQueue(uint32_t id, uint8_t *data, uint8_t len, ControlSend stat){
	for (int i = 0; i < CAN_QUEUE_SIZE; i++) {
		if (!sendDataCan[i].used) {
			sendDataCan[i].id = id;
			memcpy(sendDataCan[i].data, data, len);
			sendDataCan[i].len = len;
			sendDataCan[i].stat = stat;
			sendDataCan[i].used = true;
			return true;  // Успешно добавлено
		}
	}
	return false;  // Очередь полна
}

////////////////////////////////прерывание получения сообщения CAN-/////////////////////////
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      //----запсь в флей память
    	if (RxHeader.StdId == Sh_Id(CAN_ID_EEPROM)) { //--------установка адресса устройства
          Brd.Adres = RxData[0];
          Flash_WriteData(&Brd, sizeof(Brd));
          AddMessageToQueue(Sh_Id(CAN_ID_OKtx), NULL, 0, SEND_ONLY);
          HAL_NVIC_SystemReset();
      
      }else if (RxHeader.StdId == Sh_Id(CAN_ID_GET_TEMP)){  //----передача температуры 
          // Отправляем температуру в ответ
          BrdTel.temp = (int16_t)(readTemperatureLM335(Adc.FadcChanal) * 10);
          uint8_t tempData[2];
          tempData[0] = (uint8_t)(BrdTel.temp >> 8); // Старший байт
          tempData[1] = (uint8_t)(BrdTel.temp & 0xFF); // Младший байт
          AddMessageToQueue(Sh_Id(CAN_ID_TEMP), tempData, 2, SEND_ONLY);
      
      }else if (RxHeader.StdId == Sh_Id(CAN_ID_SET_DIM1)) { //----передача состояния диммеров
        //--------------------(пакет RxData [0] - состояние, [1] - яркость [2] - время наростания)
          //-----преобразование яркости процентов в значение MAX_DIM и MIN_DIM
          uint8_t light = map(RxData[1], 0, 100, MIN_DIM, MAX_DIM);
          uint8_t startValue =  map(RxData[3], 0, 100, MIN_DIM, MAX_DIM); // Начальное значение для плавного изменения
          if (light < MAX_DIM) light = MAX_DIM; // Ограничение минимального значения
          if (light > MIN_DIM) light = MIN_DIM; // Ограничение максимального
          lightSet(CH_DIM_1, light, RxData[2], startValue);
          // Отправляем подтверждение
          AddMessageToQueue(Sh_Id(CAN_ID_OKtx), NULL, 0, SEND_ONLY);
     
      }else if (RxHeader.StdId == Sh_Id(CAN_ID_SET_DIM2)) {
          //--------------------(пакет RxData [0] - состояние, [1] - яркость [2] - время наростания)
          //-----преобразование яркости процентов в значение MAX_DIM и MIN_DIM
          uint8_t light = map(RxData[1], 0, 100, MIN_DIM, MAX_DIM);
          uint8_t startValue =  map(RxData[3], 0, 100, MIN_DIM, MAX_DIM); // Начальное значение для плавного изменения
          if (light < MAX_DIM) light = MAX_DIM; // Ограничение минимального значения
          if (light > MIN_DIM) light = MIN_DIM; // Ограничение максимального
          lightSet(CH_DIM_2,  light, RxData[2], startValue);
          // Отправляем подтверждение 
          AddMessageToQueue(Sh_Id(CAN_ID_OKtx), NULL, 0, SEND_ONLY);
      
      }else if (RxHeader.StdId == Sh_Id(CAN_ID_EN_LIGHT)){
          lightEn(RxData[0], RxData[1]);
      }else if (RxHeader.StdId == Sh_Id(CAN_ID_OKrx)) {
          ackReceived = true;  // Устанавливаем флаг получения ACK
      } 
          
    }
}


void ProcessCANQueue(){
	for (int i = 0; i < CAN_QUEUE_SIZE; i++) {
		if (sendDataCan[i].used) {
			Send_CAN_Message(sendDataCan[i].id, sendDataCan[i].data, sendDataCan[i].len, sendDataCan[i].stat);
			sendDataCan[i].used = false;  // Освобождаем запись после отправки
			break;  // Отправляем одно сообщение за вызов
		}
	}
}

uint8_t Send_CAN_Message(uint32_t id, uint8_t *data, uint8_t len, ControlSend stat)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    int retryCount = 0;
    ackReceived = false;

    TxHeader.StdId = id;
    TxHeader.IDE   = CAN_ID_STD;
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    while (!ackReceived && retryCount < 5) {
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
            Error_Handler(); // очередь переполнена
        }
       // break;
        if (stat == SEND_ONLY)
        	break;

        uint32_t start = HAL_GetTick();
        while ((HAL_GetTick() - start) < ACK_TIMEOUT) {
            if (ackReceived) break;  // получили ответ
        }

        if (!ackReceived) {
            retryCount++;
        }
    }

    // Ждём окончания передачи, чтобы не засорять очередь
    uint32_t waitStart = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan, TxMailbox)) {
      if (HAL_GetTick() - waitStart > 50) break; // 10 мс
    }
    return ackReceived;
}


void CAN_FilterConfig(uint8_t FilterBank, uint16_t startID, uint16_t mask)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = FilterBank;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (startID << 5) & 0xFFFF;     // startID = 0x200
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (mask << 5) & 0xFFFF;    // mask = 0x7F0
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    /* Обработка ошибок CAN */
    uint32_t error = HAL_CAN_GetError(hcan);
    //printf("CAN Error: 0x%X\r\n", error);

    if (error & HAL_CAN_ERROR_BOF)
    {
        /* Обработка состояния Bus Off */
        //printf("CAN Bus Off error occurred\r\n");
        /* Здесь вы можете попытаться перезапустить CAN контроллер */
        HAL_CAN_Stop(hcan);
        HAL_CAN_Start(hcan);
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR);
    }else if(error & HAL_CAN_ERROR_ACK){
    	// ошибка получения пакета
    }
}

uint32_t Sh_Id(uint32_t id){
	return id + (Brd.Adres * SHIFT_ID);
}
/* USER CODE END 1 */
