/*
 * tSectorHandler.c
 *
 *  Created on: 29 авг. 2017 г.
 *      Author: walery
 */

//--------------

#include <FreeRTOS.h>
#include <task.h>
#include "priorities.h"

#include "inc/typedefs.h"
#include "msmotor/ms_model.h"

#include "mSegmentQuee.h"
#include "tSectorHandler.h"

#include "inc/sysDrivers.h"

#include "utils/vTaskGetRunTimeStats.h"


#define SECTOR_HANDLER_STACK_SIZE   64

//------------- defs

//-------------- vars
TaskHandle_t sectorHandling;
volatile uint32_t counter =0;
//static uint32_t be_portf1;
//static TaskStatus_t shStatus;
static const char* taskname = "SectorHandler";

#define TASK_ARRAY_SIZE 4
#define pulTotalRunTime NULL

#pragma NOINIT(pxTaskStatusArray)
TaskStatus_t pxTaskStatusArray[5];


//-------------- function

static void taskSectorhandler(void* params){
    static uint32_t eventCounter;
    volatile UBaseType_t uxArraySize, x;
    volatile uint8_t memf_counter;

//    unsigned long ulTotalRunTime, ulStatsAsPercentage;

    while(1){

//        eventCounter = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
       if( xSemaphoreTake(memf_semaphor_handler,portMAX_DELAY) == pdTRUE)
       {
           /* The mutex was successfully obtained so the shared resource can be
            * accessed safely. */
           NoOperation;
           ms_nextSector();    // Индикация

       }
//        vTaskDelay(5);

//        eventCounter = 1;

        if(eventCounter != 0){
            counter++;
            ms_nextSector();    // Индикация
            memf_counter = memf_release();
            if(memf_counter == SEGMENT_QUEE_SIZE){
                // Очередь пуста
                NoOperation;
            }
            NoOperation;

        }else{

//            vTaskGetRunTimeStats(statsBuffer);
            uxTaskGetSystemState(pxTaskStatusArray, TASK_ARRAY_SIZE, pulTotalRunTime);

//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
//            uxTaskGetSystemState();
//            vTaskDelay(500);
//            while(1){
                NoOperation;
//            }
        }


    }//task routine
}

    //
    // Create the TaskSectorHandler task.
    //

int32_t createTaskSectorHandler(void){
    if(xTaskCreate(taskSectorhandler, taskname, SECTOR_HANDLER_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SectorHandler_TASK, &sectorHandling) != pdPASS)
    {
        //Error
        return(1);
    }else{
        createSegmentQuee();
        return (0); //0
    }

}

