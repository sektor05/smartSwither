/*
 * Основной файл прошивки для умного переключателя на базе STM32F103
 * Реализует управление диммерами, обработку входов, CAN-коммуникацию и измерение температуры
 */

#include <Arduino.h>
#include "myBoard.h"
#include <Wire.h>
#include <cstdint>
#include <eXoCAN.h>
#include <iterator>
#include <stm32f1xx.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_gpio.h>
// Явные объявления функций memcpy, memset, memcmp для обхода проблем компиляции

// Создание экземпляра класса платы
myBoard board;

//------------------входные переменные--------------------------------
// Переменные для хранения предыдущих состояний входов
bool prevIn1, prevIn2, prevIn3, prevIn4;
// Переменные для хранения текущих состояний входов
bool in1, in2, in3, in4;

// Объявление аппаратных таймеров для различных задач
HardwareTimer timer(TIM2), timPWM(TIM1), timTemp(TIM3);

// Переменные для CAN-соединения
int id = 0, fltIdx = 0;

// Пины для управления диммерами
const uint16_t dimPins[] = {DIM_PIN, DIM2_PIN};
bool sendFlag = false;
// Буфер для отслеживания изменений в EEPROM (больше не нужен)
// uint8_t flagDataEEPROM[8];
//-----------------------------------------------------
// Буфер для приёма CAN-сообщений
uint8_t rxbytes[8];

int indexAdres = 0;
// Интервал отправки данных о температуре (30 секунд)
#define SEND_TEMP_TIME 30000 // sec

//---------------------------------счётчики------------------------------------------
uint32_t count = 0;
uint16_t tic = 0;

//----------------------------------температура--------------------------------------
#define maxSensors 1
// Массив для хранения значений температуры (в десятых долях градуса)
uint16_t temp[maxSensors];
bool flagD = false;

// Флаги для отслеживания изменений EEPROM (больше не нужны, т.к. запись идет в прерывании)
// bool eepromChanged = false;
// uint32_t lastEepromSave = 0;
// #define EEPROM_SAVE_DELAY 1000 // ms

///////////////////////////////объявления функций///////////////////////////////////////////////
void detect_down();
void detect_up();

/**
 * @brief Чтение температуры с датчика LM335
 * @return Температура в градусах Цельсия
 *
 * Функция считывает аналоговое значение с датчика LM335,
 * преобразует его в напряжение, а затем в температуру по формуле:
 * T(°C) = (V * 100) - 273.15
 */
double readTemperatureLM335() {
   // Считываем сырое значение с АЦП (12-бит, диапазон 0-4095)
   int raw = analogRead(ONE_W);
   // Преобразуем сырое значение в напряжение (опорное 3.3В)
   double voltage = raw * (3.3f / 4095.0f);
   // Преобразуем напряжение в температуру по формуле датчика LM335
   double temperatureC = (voltage * 100) - 273.15f;
   return temperatureC;
}

//------------------------обработчик прерывания CAN-------------------------------------
/**
 * @brief Обработчик прерывания для приёма CAN-сообщений
 *
 * Функция обрабатывает входящие CAN-сообщения различных типов:
 * - EEPROM: сохранение конфигурации
 * - OK: подтверждение получения
 * - PWM1/PWM2: управление ШИМ-каналами
 */
void canRecive() {
    // Проверяем наличие принятых данных
    uint8_t _data[1] = {1};
    if(board.receive(id, fltIdx, rxbytes) > 0) {
       
       // Обработка сообщения для записи в EEPROM
       if(id == (CAN_ID_EEPROM + board.addres)) {
          // Копируем данные в буфер
          memcpy(board.dataEPPROM, rxbytes, sizeof(board.dataEPPROM));
          // Прямая запись в EEPROM из прерывания
          board.saveDataToEPPROM(board.dataEPPROM);
          // Отправляем подтверждение приёма
          board.transmit(CAN_ID_OKtx, _data, sizeof(_data));
          // Перезагрузка микроконтроллера без задержки
          NVIC_SystemReset();
          
       } else if(id == (CAN_ID_OKrx + board.addres)) {
          // Обработка подтверждения получения
          if(_data[0]) board.canResponseFlag = true;
          
       } else if(id == (CAN_ID_PWM1 + board.addres)) {
          // Управление первым ШИМ-каналом
          // rxbytes[0] - целевое значение ШИМ (%)
          // rxbytes[1] - время перехода (мс)
          // rxbytes[2] - начальное значение ШИМ (%)
          board.PWM(0, rxbytes[0], rxbytes[1], rxbytes[2]);
          board.transmit(CAN_ID_OKtx + board.addres, _data, sizeof(_data));
          
       } else if(id == (CAN_ID_PWM2 + board.addres)) {
          // Управление вторым ШИМ-каналом
          board.PWM(1, rxbytes[0], rxbytes[1], rxbytes[2]);
          board.transmit(CAN_ID_OKtx + board.addres, _data, sizeof(_data));
       }
    }
}


//-----------обработчики прерываний по таймеру----------------
/**
 * @brief Обновление значений ШИМ
 *
 * Вызывается по прерыванию таймера для плавного изменения значений ШИМ
 */
void updatePWM() {
   board.updatePWM();
}

/**
 * @brief Отправка данных о температуре по CAN
 *
 * Вызывается по прерыванию таймера с периодом SEND_TEMP_TIME
 */
void sendTempCan() {
   double tempC = readTemperatureLM335();
   // Сохраняем температуру в десятых долях градуса для точности
   temp[0] = static_cast<int16_t>(tempC * 10);
   board.canTX16(CAN_ID_TEMP + board.addres, temp, 1); // 1 датчик, не 2
}

/**
 * @brief Прерывание для управления диммером (без нейтрали)
 *
 * Управляет моментом открытия симистора в зависимости от фазы переменного тока
 */
void isr() {
   tic++;

   for (int i = 0; i < 2; i++) {
      // Проверяем, находится ли значение диммера в допустимом диапазоне
      if ((board.dimmer[i] < MIN_DIM) && (board.dimmer[i] > MAX_DIM)) {
         // В нужный момент времени открываем симистор
         if (tic == board.dimmer[i]) {
            HAL_GPIO_WritePin(GPIOB, dimPins[i], GPIO_PIN_SET);
         } else if (tic >= board.dimmer[i] + 2) {
            // Закрываем симистор через 2 тика после открытия
            HAL_GPIO_WritePin(GPIOB, dimPins[i], GPIO_PIN_RESET);
            // Для второго канала останавливаем таймер
            if (i == 1) timer.pause();
         }
      } else {
         // Если значение вне диапазона, устанавиваем постоянное состояние
         GPIO_PinState st = (board.dimmer[i] == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
         HAL_GPIO_WritePin(GPIOB, dimPins[i], st);
      }
   }
}

/**
 * @brief Обработчик перехода через ноль (detect_up)
 *
 * Сбрасывает счётчик и перезапускает таймер при обнаружении перехода через ноль
 */
void detect_up() {
   tic = 0;
   // Сбрасываем оба выхода диммера
   HAL_GPIO_WritePin(GPIOB, dimPins[1], GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, dimPins[0], GPIO_PIN_RESET);
   timer.setCount(0);
   timer.refresh();
   timer.resume();
}

/**
 * @brief Прерывание для CAN-шины
 */
void canISR() {
    canRecive();
}

/**
 * @brief Инициализация системы
 *
 * Настраивает все периферийные устройства: CAN, таймеры, прерывания
 */
void setup() {
   // Инициализация платы и CAN-интерфейса
   board.initBoard();
   board.attachInterrupt(canISR);
   
   //-----------------настройка прерываний-----------------------
   // Прерывание по переходу через ноль (детектор синхронизации)
   attachInterrupt(SINC, detect_up, RISING);
   
   // Настройка таймера для плавного изменения ШИМ
   // TODO: Исправить прерывания таймера для совместимости с текущей версией библиотеки
   // timPWM.setMode(1, TIMER_OUTPUT_COMPARE);
   // timPWM.setCompare(1, 0);
   timPWM.attachInterrupt(updatePWM);
   timPWM.setOverflow(MIN_DIM, HERTZ_FORMAT);
   timPWM.resume();

   // Настройка таймера для периодической отправки температуры
   // TODO: Исправить прерывания таймера для совместимости с текущей версией библиотеки
   // timTemp.setMode(1, TIMER_OUTPUT_COMPARE);
   // timTemp.setCompare(1, 0);
   timTemp.attachInterrupt(sendTempCan);
   timTemp.setOverflow(SEND_TEMP_TIME*1000, MICROSEC_FORMAT);
   timTemp.resume();
   
   // Настройка таймера для управления диммером (40 мкс период)
   // TODO: Исправить прерывания таймера для совместимости с текущей версией библиотеки
   // timer.setMode(1, TIMER_OUTPUT_COMPARE);
   // timer.setCompare(1, 0);
   timer.attachInterrupt(isr);
   timer.setOverflow(40, MICROSEC_FORMAT);
   
   // Инициализация значений диммеров (выключены)
   board.dimmer[0] = 255;
   board.dimmer[1] = 255;

   // Первоначальная отправка температуры при старте
   double tempC = readTemperatureLM335();
   temp[0] = static_cast<int16_t>(tempC * 10);
   board.canTX16(CAN_ID_TEMP + board.addres, temp, 1);
   
   // Буфер flagDataEEPROM больше не нужен, т.к. запись в EEPROM происходит в прерывании
}

uint32_t last = 0;

/**
 * @brief Основной цикл программы
 *
 * Обрабатывает входы, управляет ШИМ, обрабатывает CAN-сообщения
 * и сохраняет изменения в EEPROM при необходимости
 */
void loop() {
   //------------чтение и отправка состояний входов----------------------------
   board.controlIn();  // Считываем состояния входов
   board.processCan(); // Обрабатываем очередь CAN-сообщений
   // Отправляем состояния входов по CAN
   board.canTX(CAN_ID_IN + board.addres, board.in, sizeof(board.in));
   
   // Управление ШИМ на основе состояний входов (локальный режим)
   if(!board.serverAvail) {
      // Если хотя бы один вход активен (низкий уровень), включаем ШИМ
      if(!board.in[0] || !board.in[1] || !board.in[2]) {
         board.PWM(0, 100, 0, 0); // Включаем первый канал на 100%
         board.PWM(1, 100, 0, 0); // Включаем второй канал на 100%
      } else {
         board.PWM(0, 0, 0, 0);   // Выключаем первый канал
         board.PWM(1, 0, 0, 0);   // Выключаем второй канал
      }
   }

   // Сохранение в EEPROM теперь происходит напрямую в прерывании canRecive()
   // Этот код больше не нужен
   
   delay(10); // Небольшая задержка для стабильности работы
}