/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

#define SHIFT_ID 10
  //------------------RX CAN parameters-----------------------
#define CAN_ID_GET_TEMP 800
#define CAN_ID_SET_DIM1 801
#define CAN_ID_SET_DIM2 802
#define CAN_ID_EEPROM 803
#define CAN_ID_OKrx 804
#define CAN_ID_EN_LIGHT 805
        //-------TX parameters---------------------
#define CAN_ID_IN 806
#define CAN_ID_TEMP  807
#define CAN_ID_OKtx 808


typedef enum {
	ACK_TIMEOUT = 300,
	MAX_RETRY = 3,
    WAIT_FOR_ACK,   // Ожидание подтверждения
	SEND_ONLY    // Только отправка без ожидания подтверждения
}ControlSend;

typedef struct {
    uint32_t id;
    uint8_t data[8];  // Максимальный размер данных в CAN — 8 байт
    uint8_t len;
    ControlSend stat;
    bool used;        // Флаг, указывающий, занята ли данная запись
} CANMessage;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
bool AddMessageToQueue(uint32_t id, uint8_t *data, uint8_t len, ControlSend stat);
void ProcessCANQueue();
uint8_t Send_CAN_Message(uint32_t id, uint8_t *data, uint8_t len, ControlSend stat);
void CAN_FilterConfig(uint8_t FilterBank, uint16_t startID, uint16_t mask);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
uint32_t Sh_Id(uint32_t id);
/* Debug helpers */
void CAN_AcceptAllFilter(void);
extern volatile uint32_t can_rx_count;
extern volatile uint32_t can_last_rx_id;
extern volatile uint32_t can_error_code;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

