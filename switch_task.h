//*****************************************************************************
//
// switch_task.h - Prototypes for the switch task.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#ifndef __SWITCH_TASK_H__
#define __SWITCH_TASK_H__

#include <FreeRTOS.h>
#include <task.h>


//---------------- defs
#define ENDER_XMIN_HANDLE   (1<<0)
#define ENDER_XMAX_HANDLE   (1<<1)
#define ENDER_YMAX_HANDLE   (1<<2)

//---------- vars
extern TaskHandle_t switchTaskHandle;

//*****************************************************************************
//
// Prototypes for the switch task.
//
//*****************************************************************************
extern uint32_t SwitchTaskInit(void);

#endif // __SWITCH_TASK_H__
