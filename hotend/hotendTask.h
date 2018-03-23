/*
 * hotend_taask.h
 *
 *  Created on: 22 мар. 2018 г.
 *      Author: walery
 */

#ifndef HOTEND_HOTENDTASK_H_
#define HOTEND_HOTENDTASK_H_

#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

//------------- defs

//-------------- vars
extern TaskHandle_t hotendHadling;

//--------------- function
extern uint32_t createtask_hotend(void);

// Заданная температура.
extern void setTargetHotendTemperature(float_t temp);

extern float_t getTargetHotendTemperature(void);

extern void setCurrentHotendTemperature(float_t temp);

extern float_t getCurrentHotendTemperature(void);

extern void start_hotende(void);

extern void stop_hotend(void);

#endif /* HOTEND_HOTENDTASK_H_ */
