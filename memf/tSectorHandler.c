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



#define SECTOR_HANDLER_STACK_SIZE   64

//------------- defs

//-------------- vars
TaskHandle_t sectorHandling;

//-------------- function

static void taskSectorhandler(void* params){
    static uint32_t eventCounter;
    while(1){

        eventCounter = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        if(eventCounter != 0){
                ms_nextSector();    // Индикация
                if(memf_release()>SEGMENT_QUEE_SIZE){
                    NoOperation;
                    // Очередь пуста; пополнение отстаёт от обработчика Сегментов. Беда.
                    // Хост должен снизить скорость перемещения инструмента.
                }
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

int32_t createTaskSectorHandler(void){
    if(xTaskCreate(taskSectorhandler, (const portCHAR *)"SectorHandler", SECTOR_HANDLER_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SectorHandler_TASK, &sectorHandling) != pdPASS)
    {
        //Error
        return(1);
    }else{
        createSegmentQuee();
        return (0); //0
    }

}

