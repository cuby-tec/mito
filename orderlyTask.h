/*
 * orderlyTask.h
 *
 *  Created on: 18 июн. 2017 г.
 *      Author: walery
 */

#ifndef ORDERLYTASK_H_
#define ORDERLYTASK_H_

//#include <sysbiosHeader.h>

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define X_axis_int      (1) // from axisX_intrrupt_handler to orderlyTask
#define X_axis_int_fin  (1<<1) // from axisX_intrrupt_handler to orderlyTask
#define SignalUSBbufferReady (1<<2) // Получена команда по каналу USB.
#define ot_sgQueueEmpty     (1<<3) //выполнена ОБработка всех сегментов и новых сегментов нет.
#define ot_sgTest       (1<<4)
#define ender_xmin_test (1<<5)

/**
 * 3 - check flag
 * 2 - send by IRQ59
 * 1 - send response from orderlyTask
 * 0 - send it from usbmodule.
 */
#define  sendStatus_p   4


//---------- vars

extern TaskHandle_t orderlyHandling;
extern xQueueHandle orderlyQueue;

//--------- Function
extern uint32_t createtask_orderly(void);

extern void IntIRQ59(void);


#endif /* ORDERLYTASK_H_ */
