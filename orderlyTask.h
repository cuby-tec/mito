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


//---------- vars
extern TaskHandle_t orderlyHandling;

//--------- Function
extern uint32_t createtask_orderly(void);


#endif /* ORDERLYTASK_H_ */
