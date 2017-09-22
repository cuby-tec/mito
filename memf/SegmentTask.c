/*
 * SegmentTask.c
 *
 *  Created on: 15 сент. 2017 г.
 *      Author: walery
 */

//--------------

#include <FreeRTOS.h>
#include <task.h>
#include <limits.h>
#include "priorities.h"

#include "inc/typedefs.h"
#include "SegmentTask.h"
#include "mSegmentQuee.h"
#include "msmotor/mempool.h"

//------------- defs
#define SEGMENT_TASK_STACK_SIZE     64
#define SEGMENT_NOTIFY_DELAY    portMAX_DELAY
//-------------- vars

TaskHandle_t segmentHandling;

static const char* pcName = "SegmentTask";


//-------------- function

static void segmentHandler(void* params){
    uint32_t ulNotifiedValue;
    BaseType_t ret;

    while(1){
        ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, SEGMENT_NOTIFY_DELAY );//portMAX_DELAY

        if(SECTOR_RECEIVED & ulNotifiedValue){

            //        eventCounter = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            // rcvd_semaphore_handler
            if(xSemaphoreTake(rcvd_semaphore_handler,portMAX_DELAY) == pdTRUE)
            {
                /* The mutex was successfully obtained so the shared resource can be
                 * accessed safely. */
                uint8_t* ss = (uint8_t*)MEMF_Alloc();
//                memcpy(ss, segmentBuffer, sizeof(struct sSegment));
                xSemaphoreGive(rcvd_semaphore_handler);

                NoOperation;
//                ms_nextSector();    // Индикация

            }
            //        vTaskDelay(5);

            //        eventCounter = 1;
        }

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

