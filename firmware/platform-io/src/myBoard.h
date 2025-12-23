
/*
 * Заголовочный файл класса myBoard
 * Определяет функционал для управления платой умного переключателя
 * Включает управление диммерами, CAN-коммуникацию, работу с EEPROM
 */

#ifndef MY_BOARD_H
#define MY_BOARD_H

/*
 * Заголовочный файл класса myBoard
 * Определяет функционал для управления платой умного переключателя:
 * - Управление диммерами и ШИМ
 * - CAN-коммуникация
 * - Работа с EEPROM
 * - Обработка входов
 */

#include <Arduino.h>
#include "EEPROM.h"
#include <eXoCAN.h>
#include <stm32f1xx.h>
#include <cstring>
#include <stm32f1xx_hal.h>

// Явные объявления функций memcpy, memset, memcmp для обхода проблем компиляции
extern "C" {
void *memcpy(void *dest, const void *src, size_t n);
void *memset(void *s, int c, size_t n);
int memcmp(const void *s1, const void *s2, size_t n);
}
//------------------определения пинов-------------------

// Пины управления диммерами
#define DIM_PIN PB8
#define DIM2_PIN PB9

// Входные пины
#define IN_1 PA0
#define IN_2 PA1
#define IN_3 PA2

// Пин детектора перехода через ноль (синхронизация)
#define SINC PA3

// CAN-шина пины
#define CAN_RX PA11
#define CAN_TX PA12

// I2C пины
#define I2C_SCL PB6
#define I2C_SDA PB7

// Пины для датчиков
#define ONE_W PA5
#define TEMP_SENSOR_PIN PA5  // Пин, к которому подключён LM335Z

// Адрес в EEPROM для хранения конфигурации
#define CONFIG_ADDR 10

// Пределы диммера (значения для таймера)
#define MIN_DIM 200  // Минимальное значение (максимальная яркость)
#define MAX_DIM 0    // Максимальное значение (минимальная яркость)

//------------------параметры CAN-сообщений (приём)-----------------------
#define CAN_ID_GLOBAL 800  // Глобальный CAN ID
#define CAN_ID_PWM1 801    // Управление первым ШИМ-каналом
#define CAN_ID_PWM2 802    // Управление вторым ШИМ-каналом
#define CAN_ID_EEPROM 803  // Запись в EEPROM
#define CAN_ID_OKrx 804    // Подтверждение приёма

//-------параметры CAN-сообщений (передача)---------------------
#define CAN_ID_IN 806      // Состояния входов
#define CAN_ID_TEMP  807   // Датчик температуры
#define CAN_ID_OKtx 808    // Подтверждение передачи

//----------------индексы данных в EEPROM-----------------
#define CONFIG 0          // Конфигурация устройства


//---------------типы конфигурации устройства-----------------------
#define COSTOM 0          // Пользовательский режим
#define POWER 1           // Режим управления питанием
#define RESTROOM 2        // Режим для ванной комнаты
#define KICHEN 3          // Режим для кухни


/**
 * @brief Класс управления платой умного переключателя
 *
 * Наследуется от eXoCAN для обеспечения CAN-коммуникации.
 * Реализует управление диммерами, ШИМ, обработку входов и работу с EEPROM.
 */
class myBoard: public eXoCAN {
    public:
        // Публичные переменные
        uint8_t config=0;              // Тип конфигурации устройства
        uint8_t addres=0;              // Адрес устройства в CAN-сети
        uint8_t led2=0, led3=0, led4=0; // Состояния светодиодов
        
        uint8_t in[4];                 // Состояния входов
        uint8_t dataEPPROM[8];         // Данные из EEPROM
        uint8_t pwm_balast=0;          // Значение ШИМ балласта для первой лампы, когда включена вторая лампа. Необходима для детектора нуля.
        bool serverAvail = true;       // Флаг доступности сервера CAN

        // Публичные методы
        void initBoard();              // Инициализация платы
        bool contrCheng(int &previousData, int data); // Проверка изменения данных
       
        bool controlIn();              // Контроль входов
        uint8_t IN1();                 // Чтение состояния входа 1
        uint8_t IN2();                 // Чтение состояния входа 2
        uint8_t IN3();                 // Чтение состояния входа 3
        uint8_t dimmer[2];             // Значения диммеров
       
        
        uint8_t PWM(uint8_t chinal, uint8_t pwm, uint16_t time, uint8_t startPercent); // Управление ШИМ с плавным изменением
     
        void exponentialPWM(uint8_t pin, uint16_t duration); // Экспоненциальное изменение ШИМ
        void PWM(uint8_t pin, uint8_t percent);               // Простое управление ШИМ

        // CAN-методы
        bool canTX(int txId, uint8_t *ptr, uint8_t len, bool waitAck = true); // Отправка CAN-сообщения с ожиданием подтверждения
        bool canTX16(int txId, uint16_t *ptr, uint8_t len);                    // Отправка 16-битных данных по CAN
        bool canTX16_notAks(int txId, uint16_t *ptr, uint8_t len);            // Отправка 16-битных данных без подтверждения
        bool canTX_notCont(int txId, const void *ptr, uint8_t len);           // Отправка без непрерывного подтверждения
        void processCan(); // Обработка очереди CAN-сообщений (неблокирующая)

        // Методы работы с EEPROM
        void saveDataToEPPROM(const uint8_t data[8]); // Сохранение данных в EEPROM
        void getDataToEPPROM();                       // Чтение данных из EEPROM
        void updatePWM();                              // Обновление значений ШИМ
        void savePwmBalastToEPPROM(uint8_t pwmBalastPerzent); // Сохранение значения балласта в EEPROM
        
        // Флаг для внешнего обработчика CAN-приёма
        // Внешний canRecive() должен установить этот флаг в true
        volatile bool canResponseFlag = false;

   private:
       /**
        * @brief Класс для фильтрации дребезга контактов
        */
       class DebounceFilter {
           private:
               unsigned long lastChangeTime;
               uint8_t lastState;
               uint8_t stableState;
               static const unsigned long DEBOUNCE_DELAY = 100; // 50мс для фильтрации дребезга
               
           public:
               DebounceFilter() : lastChangeTime(0), lastState(HIGH), stableState(HIGH) {}
               
               uint8_t filterInput(uint8_t pin) {
                   uint8_t currentState = digitalRead(pin);
                   unsigned long currentTime = millis();
                   
                   // Если состояние изменилось, запоминаем время
                   if (currentState != lastState) {
                       lastChangeTime = currentTime;
                       lastState = currentState;
                   }
                   
                   // Если прошло достаточно времени с последнего изменения, обновляем стабильное состояние
                   if (currentTime - lastChangeTime > DEBOUNCE_DELAY) {
                       stableState = currentState;
                   }
                   
                   return stableState;
               }
       };
       
       // Приватные переменные и структуры
       DebounceFilter _inputFilter1, _inputFilter2, _inputFilter3;
       
        
        /**
         * @brief Структура для управления ШИМ-каналом
         */
        struct PWMChannel {
            uint8_t pin;              // Пин управления
            uint8_t targetBrightness; // Целевая яркость
            uint8_t updateRate;       // Скорость изменения
            uint32_t counter;         // Счётчик для плавного изменения
            uint16_t setPwm;          // Устанавливаемое значение ШИМ
            uint16_t pwm;             // Текущее значение ШИМ
            uint16_t startPwm;        // Начальное значение ШИМ
        };
        PWMChannel channels[2];      // Два ШИМ-канала
        
        unsigned long _last2, _last3; // Таймеры для светодиодов
        bool flagLed1 = false, flagLed2 = false, flagLed3 = false; // Флаги светодиодов
        
        // Приватные методы
        void _delayLED2();            // Задержка для светодиода 2
        void _delayLED3();            // Задержка для светодиода 3
        void _initEEPROM();           // Инициализация EEPROM
        uint8_t getChinalsIndex(uint16_t chinals); // Получение индекса канала
        bool _prevIn[4];              // Предыдущие состояния входов
        void CAN_FilterConfig(uint8_t FilterBank, uint16_t startID, uint16_t mask); // Конфигурация CAN-фильтров

        // Очередь передачи CAN-сообщений
        static const uint8_t CAN_QUEUE_SIZE = 16;
        struct CanPacket {
            int txId;                  // ID сообщения
            uint8_t data[8];           // Данные сообщения
            uint8_t len;               // Длина данных
            uint8_t retries;           // Количество повторных отправок
            unsigned long lastSentMs;  // Время последней отправки
            bool waitAck;              // Флаг ожидания подтверждения
        };
        CanPacket _canQueue[CAN_QUEUE_SIZE]; // Очередь CAN-пакетов
        uint8_t _canQueueHead = 0;     // Индекс следующего пакета для обработки
        uint8_t _canQueueTail = 0;     // Индекс для добавления нового пакета
        uint8_t _canQueueCount = 0;    // Количество пакетов в очереди

        // Состояние пакета в процессе передачи
        bool _canInFlight = false;    // Флаг наличия пакета в процессе передачи
        int _canInFlightId = -1;       // ID пакета в процессе передачи
        unsigned long _canInFlightStart = 0; // Время начала передачи
        uint8_t _canInFlightRetries = 0;     // Количество повторов
        uint8_t _canInFlightLen = 0;          // Длина данных пакета
        uint8_t _canInFlightData[8];          // Данные пакета в процессе передачи

        // Вспомогательная функция для добавления в очередь
        bool _enqueueCanPacket(int txId, const uint8_t* data, uint8_t len, bool waitAck);

        // Массивы для отслеживания предыдущих CAN-сообщений
        int _prevId[20];               // Предыдущие ID сообщений
        uint8_t _prevData[20][8];      // Предыдущие данные сообщений
        uint8_t endIndexID=0;          // Индекс для циклического буфера
        int getIndex(int id);          // Получение индекса по ID
        int setIndex(int id);           // Установка индекса для ID

      
        /**
         * @brief Преобразование 16-битного значения в массив байт
         * @param value Входное 16-битное значение
         * @param byteArray Выходной массив байт
         * @param index Индекс начала записи в массив
         */
        void _uint16ToByteArray(uint16_t value, uint8_t* byteArray, uint8_t index) {
            byteArray[index] = static_cast<uint8_t>(value & 0xFF);         // Младший байт
            byteArray[index+1] = static_cast<uint8_t>((value >> 8) & 0xFF);  // Старший байт
        }
        
        /**
         * @brief Преобразование массива байт в 16-битное значение
         * @param byteArray Входной массив байт
         * @param index Индекс начала чтения из массива
         * @return 16-битное значение
         */
        uint16_t byteArrayToUint16(const uint8_t* byteArray, uint8_t index) {
            uint16_t result = 0;
            result |= static_cast<uint16_t>(byteArray[index]);      // Младший байт
            result |= static_cast<uint16_t>(byteArray[index + 1]) << 8;  // Старший байт
            return result;
        }


};


void myBoard::initBoard(){
    
    // Включаем тактирование для порта GPIOB, необходимое для HAL-функций
    __HAL_RCC_GPIOB_CLK_ENABLE();

    _initEEPROM();
    //saveDataToEPPROM(0);
    begin(STD_ID_LEN, BR125K, PORTA_11_12_WIRE_PULLUP); // 
    setAutoTxRetry(true);  
    // Configure CAN hardware filters: accept only the IDs we care about
    // Bank 0: list of four standard 16-bit IDs (800..803)
    //filterList16Init(0, CAN_ID_OKrx, CAN_ID_PWM2, CAN_ID_EEPROM);
    filterMask32Init(0, (uint32_t)(CAN_ID_GLOBAL + addres), 0x1fffffff);
    filterMask32Init(1, (uint32_t)(CAN_ID_PWM1 + addres), 0x1fffffff);
    filterMask32Init(2, (uint32_t)(CAN_ID_PWM2 + addres), 0x1fffffff);
    filterMask32Init(3, (uint32_t)(CAN_ID_EEPROM + addres), 0x1fffffff);
    filterMask32Init(4, (uint32_t)(CAN_ID_OKrx + addres), 0x1fffffff);
    // Bank 1: accept OKrx (804)
    //filterMask32Init(0, ((CAN_ID_OKrx << 5) & 0xFFFF), ((0x07FF << 5) & 0xFFFF));
    //filterMask16Init(0, CAN_ID_OKrx, 0x07F0, CAN_ID_OKrx, 0x07F0); // accept 4 IDs starting at OKrx
    
    analogWriteFrequency(120);
    analogWriteResolution (16);
    analogReadResolution(12);  // Значения будут от 0 до 4095
 

    pinMode(DIM_PIN, OUTPUT);
    pinMode(DIM2_PIN, OUTPUT);

    digitalWrite(DIM_PIN, LOW);
    digitalWrite(DIM2_PIN, LOW);
   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

    pinMode(ONE_W, OUTPUT);
    pinMode(SINC, INPUT);
    pinMode(IN_1, INPUT_PULLUP);
    pinMode(IN_2, INPUT_PULLUP);
    pinMode(IN_3, INPUT_PULLUP);
  
    channels[0].pwm = MIN_DIM;
    channels[1].pwm = MIN_DIM;
    
}

bool myBoard::controlIn(){
    in[0] = IN1();
    in[1] = IN2();
    in[2] = IN3();
    return true;
 }

bool myBoard::contrCheng(int &previousData, int data){
    int _data = data;
    if(previousData!=_data){
        previousData = _data;
        return true;
    }
    return false;
}
bool myBoard::canTX(int txId, uint8_t *ptr, uint8_t len, bool waitAck){
    // Enqueue non-ack packet if identical to previous data prevents duplicates
    int indexId = getIndex(txId);
    const uint8_t size2 = len;
    bool shouldSend = true;
    if((indexId != -1 && indexId < 20)){
        // compare previous data
        for(uint8_t i=0; i < size2; i++){
            if(_prevData[indexId][i] != ptr[i]){
                shouldSend = true;
                break;
            }
            shouldSend = false;
        }
        if(shouldSend){
            // copy into prev buffer
            uint8_t copyLen = size2 > 8 ? 8 : size2;
            memcpy(_prevData[indexId], ptr, copyLen);
        }
        else return false;
    }
    else {
        indexId = setIndex(txId);
        uint8_t copyLen = size2 > 8 ? 8 : size2;
        memcpy(_prevData[indexId], ptr, copyLen);
    }

    // enqueue packet for sending; respect caller-supplied waitAck
    return _enqueueCanPacket(txId, ptr, size2, waitAck);
}

bool myBoard::canTX16(int txId, uint16_t *ptr, uint8_t len){
    if(len<=4){
        uint8_t _data[8];
        memset(_data, 0, sizeof(_data));
        for (uint8_t i = 0; i < len; i++){
            _uint16ToByteArray(ptr[i], _data, i*2);
        }
        return canTX(txId, _data, len*2);
    }
    return false;
}


bool  myBoard::canTX16_notAks(int txId, uint16_t *ptr, uint8_t len){
    if(len<=4){
        uint8_t _data[8];
        memset(_data, 0, sizeof(_data));
        for (uint8_t i = 0; i < len; i++){
            _uint16ToByteArray(ptr[i], _data, i*2);
        }
        return canTX_notCont(txId, _data, len*2);
    }
    return false;
}

bool myBoard::canTX_notCont(int txId, const void *ptr, uint8_t len) {
    // Enqueue a non-blocking packet that expects no continuous confirmation
    return _enqueueCanPacket(txId, (const uint8_t*)ptr, len, false);
}

// enqueue helper implementation
bool myBoard::_enqueueCanPacket(int txId, const uint8_t* data, uint8_t len, bool waitAck){
    if(len > 8) return false;
    if(_canQueueCount >= CAN_QUEUE_SIZE) return false; // full
    // copy into tail slot
    CanPacket &p = _canQueue[_canQueueTail];
    p.txId = txId;
    memset(p.data, 0, sizeof(p.data));
    for(uint8_t i=0;i<len;i++) p.data[i] = data[i];
    p.len = len;
    p.retries = 0;
    p.lastSentMs = 0;
    p.waitAck = waitAck;

    _canQueueTail = (_canQueueTail + 1) % CAN_QUEUE_SIZE;
    _canQueueCount++;
    return true;
}

// process queue: call often from loop()
void myBoard::processCan(){
    const unsigned long retryInterval = 500; // ms
    const uint8_t maxRetries = 5;

    // If there's an in-flight packet, check for ack or timeout
    if(_canInFlight){
        // Check external receive flag (set by your canRecive handler)
        if(canResponseFlag){
            // if the last received message is the ACK we expect
           {
                serverAvail = true;
                _canInFlight = false;
                _canInFlightId = -1;
                _canInFlightRetries = 0;
            }
            // reset the flag after processing
            canResponseFlag = false;
        }

        if(_canInFlight && (millis() - _canInFlightStart > retryInterval)){
            // timeout: retry or drop
            if(_canInFlightRetries < maxRetries){
                // resend
                transmit(_canInFlightId, _canInFlightData, _canInFlightLen);
                _canInFlightStart = millis();
                _canInFlightRetries++;
            } else {
                // give up
                serverAvail = false;
                _canInFlight = false;
                _canInFlightId = -1;
                _canInFlightRetries = 0;
            }
        }
        // while in-flight, do not start new packet
        if(_canInFlight) return;
    }

    // If no in-flight and queue not empty, send next packet
    if(_canQueueCount == 0) return;

    CanPacket pkt = _canQueue[_canQueueHead];
    // attempt transmit
    bool ok = transmit(pkt.txId, pkt.data, pkt.len);
    if(ok){
        if(pkt.waitAck){
            // move to in-flight
            _canInFlight = true;
            _canInFlightId = pkt.txId;
            _canInFlightLen = pkt.len;
            memcpy(_canInFlightData, pkt.data, pkt.len);
            _canInFlightStart = millis();
            _canInFlightRetries = 0;
        }
        // remove from queue
        _canQueueHead = (_canQueueHead + 1) % CAN_QUEUE_SIZE;
        _canQueueCount--;
    } else {
        // if transmit failed, mark server unavailable and try later
        serverAvail = false;
        // increment retry counter in queue (if any)
        // simple backoff: rotate tail to try later
        // do nothing else; packet stays in queue for next attempt
    }
}

uint8_t myBoard:: PWM(uint8_t chinal, uint8_t pwmPercent, uint16_t time, uint8_t startPercent){
    if (pwmPercent > 100 ) pwmPercent = 100;
    uint16_t pwm = map(pwmPercent, 0, 100, MIN_DIM, MAX_DIM);
    uint16_t startPwm =  map(startPercent, 0, 100, MIN_DIM, MAX_DIM);
   // const uint8_t rate = pwm !=0 ? (ms / div) : channels[index].timeSicl;
    //const uint32_t time_ms = (channels[index].setPwm==0 || pwm ==0) ?  (ms / pwm) : channels[index].timeSicl;
    channels[chinal].updateRate = time;
    channels[chinal].setPwm = pwm;
    channels[chinal].startPwm = startPwm;
    //channels[chinal].pwm = 170;
    channels[chinal].counter = 0;
    if (channels[chinal].updateRate != 0) {
        dimmer[chinal] = channels[chinal].pwm;
    } else {
        dimmer[chinal] = channels[chinal].setPwm; // Если плавное изменение выключено, сразу ставим нужное значение
    }
    
    // Прямое управление выходами для значений 0% и 100% (без диммера)
    if (pwmPercent == 0) {
        // 0% - выключено
        HAL_GPIO_WritePin(GPIOB, (chinal == 0) ? GPIO_PIN_8 : GPIO_PIN_9, GPIO_PIN_RESET);
    } else if (pwmPercent == 100) {
        // 100% - включено
        HAL_GPIO_WritePin(GPIOB, (chinal == 0) ? GPIO_PIN_8 : GPIO_PIN_9, GPIO_PIN_SET);
    }
    
    return pwm;
}

void myBoard::updatePWM(){

    for(int i=0; i< 2; i++){

        if((channels[i].updateRate !=0)){
            if(channels[i].counter++ < channels[i].updateRate){
                continue;
            }

            channels[i].counter = 0;
            if((channels[i].pwm < channels[i].setPwm))
                if( channels[i].startPwm <  channels[i].pwm) channels[i].pwm = channels[i].setPwm; 
                else channels[i].pwm++;
            else if((channels[i].pwm > channels[i].setPwm)){
                if( channels[i].startPwm <  channels[i].pwm) channels[i].pwm = channels[i].startPwm; 
                else channels[i].pwm--;
            }
            dimmer[i] = channels[i].pwm; 
        }

    }
}

int myBoard::getIndex(int txId){
   //size_t numElements = sizeof(_prevId) / sizeof(int);
    for(int i=0; i<20; i++){
        if(_prevId[i] == txId){
            return i;
            break;
        }        
    }
    return -1;
}

int myBoard::setIndex(int txId){
    int index = getIndex(txId);
    if(index ==-1){
        _prevId[endIndexID] = txId;
        endIndexID++;
        if(endIndexID>=20)
            endIndexID = 0;
        return endIndexID-1;
    }
    return -1;
}

uint8_t myBoard::IN1(){
    in[0] = _inputFilter1.filterInput(IN_1);
    return in[0];
}

uint8_t myBoard::IN2(){
    in[1] = _inputFilter2.filterInput(IN_2);
    return in[1];
}

uint8_t myBoard::IN3(){
    in[2] = _inputFilter3.filterInput(IN_3);
    return in[2];
}

uint8_t myBoard::getChinalsIndex(uint16_t chinals){
    switch (chinals)
    {
    case 0:
        return 0;
        break;
    case 1:
        return 1;
        break;
    default:
        return 0;
    }
 }


void myBoard::_initEEPROM(){
    getDataToEPPROM();

    if (dataEPPROM[0] != 255) {
        config = dataEPPROM[0];
        addres = config * 10;
        // load pwm_balast from stored EEPROM byte (index 1)
        pwm_balast = dataEPPROM[1];
    } else {
        // First-run defaults
        config = 0;
        addres = 0;
        pwm_balast = 0;
        // write defaults into EEPROM
        for (int i = 0; i < 8; i++) {
            uint8_t v = 0;
            if (i == 0) v = (uint8_t)config;
            if (i == 1) v = pwm_balast;
            EEPROM.write(CONFIG_ADDR + i, v);
        }
        // refresh cache
        getDataToEPPROM();
    }

}
void myBoard::getDataToEPPROM(){
    for (int i = 0; i < 8; i++) {
        dataEPPROM[i] = EEPROM.read(CONFIG_ADDR + i);
    }
    // synchronize pwm_balast with stored value
    pwm_balast = dataEPPROM[1];
}
void myBoard::saveDataToEPPROM(const uint8_t data[8]){
    for (int i = 0; i < 8; i++) {
        EEPROM.write(CONFIG_ADDR + i, data[i]);
    }
    // reload cache/fields
    getDataToEPPROM();
}

void myBoard::savePwmBalastToEPPROM(uint8_t pwmBalastPerzent){   
    EEPROM.write(CONFIG_ADDR + 1, pwmBalastPerzent);
}


#endif