/*
 * ComDataReq_t.h
 *
 *  Created on: 10 сент. 2017 г.
 *      Author: walery
 */

#ifndef EXCHANGE_COMDATAREQ_T_H_
#define EXCHANGE_COMDATAREQ_T_H_

#include "msmotor/sSegment.h"
#include "sHotendControl.h"
//------------- defs

enum eOrder{
    eoState,            // Запрос состояния устройства.
    eoHotendParams,    // Задание параметров Hotend
    eoSegment           // Семент движения.
};


struct sControlCommand_t {
    uint16_t order;      // Команда инструменту.
    uint8_t reserved:4;
    uint8_t instrument; // Тип инструмента в запросе.
};

struct ComDataReq_t{
    uint32_t    requestNumber;  // Номер запроса порядковый.
    uint8_t     size;           // Разме сообщения.
    uint8_t     instruments;    // Количество инструментов в запросе.
    struct sControlCommand_t    command;        // Команда управления контроллером
    //-------- payload
    union {
    struct sSegment instrument1_parameter;  // Сегмент движения.
    struct sHotendControl_t instrument_hotend;           // Управление инструментом Hotend.
    uint32_t    instrument2_paramter;       // или параметры инструмента 2
    uint32_t    instrument3_paramter;       // или параметры инструмента 3
    uint32_t    instrument4_paramter;       // или параметры инструмента 4
    }payload;
};


//-------------- vars


//--------------- function



#endif /* EXCHANGE_COMDATAREQ_T_H_ */
