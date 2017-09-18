/*
 * tSectorHandler.c
 *
 *  Created on: 29 авг. 2017 г.
 *      Author: walery
 */

//--------------

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "priorities.h"
#include "limits.h"
#include "inc/typedefs.h"
#include "msmotor/ms_model.h"

#include "mSegmentQuee.h"
#include "tSectorHandler.h"

#include "inc/sysDrivers.h"

#include "utils/vTaskGetRunTimeStats.h"
#include "msmotor/mempool.h"

#define SECTOR_HANDLER_STACK_SIZE   384//320//256//192//128

//------------- defs
#define SECTOR_DELAY    portMAX_DELAY
#define TASK_ARRAY_SIZE 5
#define pulTotalRunTime NULL

//-------------- vars
TaskHandle_t sectorHandling;
volatile uint32_t counter =0;
//static uint32_t be_portf1;
//static TaskStatus_t shStatus;
static const char* taskname = "SectorHandler";


#pragma NOINIT(pxTaskStatusArray)
TaskStatus_t pxTaskStatusArray[6];

/**
 * Флаг перехода на новый сегмент:
 * - TRUE - переход завершён.
 * - FALSE - переход выполняется.
 */
bool sema_tail = TRUE;

//-------------- function

static void taskSectorhandler(void* params){
//    static uint32_t eventCounter;
    volatile UBaseType_t uxArraySize, x;
    volatile UBaseType_t memf_counter;

    struct sSegment* empty_segment;

    //    unsigned long ulTotalRunTime, ulStatsAsPercentage;

    while(1){

//void* MEMF_Alloc(void)

        if( xSemaphoreTake(memf_semaphor_handler,portMAX_DELAY) == pdTRUE){
            // move pblock
            // block_buffer_tail = next_block_index(block_buffer_tail);
//            memf_counter = memf_release();
            memf_counter = uxSemaphoreGetCount(memf_semaphor_handler);
            //todo Загрузить следующий сегмент
            empty_segment = getSegment();   //(struct sSegment*)MEMF_Alloc();
            if(empty_segment != NULL){
                empty_segment->head.linenumber += SEGMENT_QUEE_SIZE;
                ms_nextSector();    // Индикация
            }else{
                // header догнал tail
//                vTaskGetRunTimeStats(statsBuffer);
                uxTaskGetSystemState(pxTaskStatusArray, TASK_ARRAY_SIZE, pulTotalRunTime);
                NoOperation;
            }
            counter++;
        }

/*
        if(eventCounter != 0){
            counter++;
//            ms_nextSector();    // Индикация
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
*/


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

