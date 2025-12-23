#include "flash_storage.h"
#include "string.h"



//---функции записи и считывания сруктуры с флеш памяти
void Flash_WriteData(void *data, uint32_t size) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = STRUCTUR_ARRESS;
    eraseInit.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return; // Ошибка стирания
    }

    for (uint32_t i = 0; i < size / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STRUCTUR_ARRESS + i * 4, ((uint32_t*)data)[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return; // Ошибка записи
        }
    }

    HAL_FLASH_Lock();
}

void Flash_ReadData(void *data, uint32_t size) {
    memcpy(data, (void*)STRUCTUR_ARRESS, size);
}
