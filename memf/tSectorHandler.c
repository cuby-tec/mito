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

//todo Загрузить следующий сегмент
// Ожидание получения доступа к буферу Сегмента приёмника.
static void semaphoreTake(void)
{
    struct sSegment* empty_segment;
    BaseType_t ret;
    ret = xSemaphoreTake(rcvd_semaphore_handler,portMAX_DELAY);
    if(ret == pdPASS){
        empty_segment = getSegment();   //(struct sSegment*)MEMF_Alloc();
        if(empty_segment != NULL){
//            empty_segment->head.linenumber += SEGMENT_QUEE_SIZE;
            memcpy((uint8_t*)empty_segment, segmentBuffer, sizeof(struct sSegment));
            rcvd_SegmentFlag = 0;
      }else{
//            header догнал tail
//                vTaskGetRunTimeStats(statsBuffer);
            uxTaskGetSystemState(pxTaskStatusArray, TASK_ARRAY_SIZE, pulTotalRunTime);
            NoOperation;
        }

        xSemaphoreGive(rcvd_semaphore_handler);
     }
}



/**
 * Копирование принятого Сегмента в свободный блок Сектора.
 */
static memf_segmentWait(void){

    BaseType_t ret;
    uint32_t ulNotifiedValue;
    // Ожидание сообщения о том, что Сегмент принят.
    ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue,portMAX_DELAY );//portMAX_DELAY
    if(ret == pdTRUE){
        if( sg_segmentRecieved & ulNotifiedValue ){
            semaphoreTake();
        }
    }
}

static void taskSectorhandler(void* params){
//    static uint32_t eventCounter;
    volatile UBaseType_t uxArraySize, x;
    volatile UBaseType_t memf_counter;

//    struct sSegment* empty_segment;

    //    unsigned long ulTotalRunTime, ulStatsAsPercentage;

    while(1){

//void* MEMF_Alloc(void)

        if( xSemaphoreTake(memf_semaphor_handler,portMAX_DELAY) == pdTRUE){
            // move pblock
            // block_buffer_tail = next_block_index(block_buffer_tail);
//            memf_counter = memf_release();
            memf_counter = uxSemaphoreGetCount(memf_semaphor_handler);
            ms_nextSector();    // Индикация

            memf_segmentWait();

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

