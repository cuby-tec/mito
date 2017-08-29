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

#include "ms_model.h"
#include "tSectorHandler.h"

#define SECTOR_HANDLER_STACK_SIZE   64

//------------- defs

//-------------- vars
TaskHandle_t sectorHandling;

//-------------- function

static void taskSectorhandler(void){
    static uint32_t eventCounter;
    while(1){

        eventCounter = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        if(eventCounter != 0){
                ms_nextSector();
            if(eventCounter == 1)
                NoOperation;
            else
                switch(eventCounter){
                case 2:
                    NoOperation;
                    break;
                case 3:
                    NoOperation;
                    break;
                case 4:
                    NoOperation;
                    break;
                default:
                    NoOperation;
                    break;
                }

        }


    }//task routine
}

    //
    // Create the TaskSectorHandler task.
    //

uint32_t createTaskSectorHandler(void){
    if(xTaskCreate(taskSectorhandler, (const portCHAR *)"SectorHandler", SECTOR_HANDLER_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SectorHandler_TASK, &sectorHandling) != pdTRUE)
    {
        return(1);
    }else{
        return (0);
    }

}

