/*
 * SegmentTask.c
 *
 *  Created on: 15 сент. 2017 г.
 *      Author: walery
 */

//--------------

#include <FreeRTOS.h>
#include <task.h>
#include "priorities.h"

#include "inc/typedefs.h"
#include "SegmentTask.h"

//------------- defs
#define SEGMENT_TASK_STACK_SIZE     64

//-------------- vars

TaskHandle_t segmentHandling;

static const char* pcName = "SegmentTask";


//-------------- function

static void segmentHandler(void* params){
    while(1){
        NoOperation;
    }// while
}


uint32_t createSegmentTask(void)
{
    if(xTaskCreate(segmentHandler, pcName, SEGMENT_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY+PRIORITY_SegmentTask, &segmentHandling) != pdPASS)
    {
        //Error
        return(1);
    }else
    {
        return(0);
    }
}

