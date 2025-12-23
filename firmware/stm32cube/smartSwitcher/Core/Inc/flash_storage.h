/**@file   flash_storege.h
 * @brief  This file contains all the function prototypes for
 *         the flash_storage.c file
 ******************************************************************************
 * @attention
 *
*/
#ifndef __FLASH_STORAGE_H__
#define __FLASH_STORAGE_H__

#include "main.h"


#define FLASH_USER_START_ADDR    0x0801F800  // Адрес начала последней страницы (128 КБ флеш - 2 КБ)
#define APPLICATION_ADDRESS 0x08004000

#define STRUCTUR_ARRESS       (FLASH_USER_START_ADDR)


#define VECT_TAB_OFFSET     0x4000    // Смещение таблицы векторов (на основе APPLICATION_ADDRESS)
#define VECT_TAB_SIZE       0x100        // Размер таблицы векторов (типичный размер для Cortex-M0)

#define BACKUP_FLAG_ADDRESS RTC_BKP_DR0  // Используем первый резервный регистр для флага



//---функции записи и считывания сруктуры с флеш памяти
void Flash_WriteData(void *data, uint32_t size);
void Flash_ReadData(void *data, uint32_t size);





#endif // FLASH_STORAGE_H
