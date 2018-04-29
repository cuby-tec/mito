/*
 * orderlyTask.c
 *
 *  Created on: 18 июн. 2017 г.
 *      Author: walery
 */

//#include <xdc/runtime/Timestamp.h>

//#include <rgb.h>
//#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_types.h"
//#include "drivers/rgb.h"
//#include "drivers/sysctl.h"
//#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
//#include "inc/sysDrivers.h"

#include "orderlyTask.h"
#include "msmotor/mempool.h"
#include "msmotor/msport.h"
#include "msmotor/ms_init.h"
#include "memf/mSegmentQuee.h"
#include "msmotor/ms_model.h"
#include "exchange/ComDataReq_t.h"

#include "priorities.h"
#include <limits.h>

#include "exchange/status.h"

#include "memf/tSectorHandler.h"

//------------- DEFS

#define ORDERLYTASKSTACKSIZE  128//640//576 //128 //192 640 //

//---------  vars

//Task_Struct taskOrderlyStruct;
//Char taskOrderlyStack[TASKSTACKSIZE];

//static UInt32 time1,time2, time3 = 0;
//static Types_FreqHzf hz;


TaskHandle_t orderlyHandling;
#pragma NOINIT(taskcounter)
uint32_t taskcounter;


static struct ComDataReq_t* msegment;

//--------- function


#define ORDERLY_DELAY   500
//static uint32_t cnt_delay;
//static uint32_t delay_max;
void orderly_routine(void* pvParameters ){
    uint32_t ulNotifiedValue;
//    volatile eTaskState state;
    BaseType_t sema, xStatus;
    BaseType_t ret;
    static uint32_t be_portf3;
    uint8_t* ss;
//    testPrepare();
//    initBlock();

    initStepper(N_AXIS);
    ms_finBlock = exitBlock;
//    start_t1(0);
    taskcounter = 0;
    for( ;; ){
//portMAX_DELAY
        ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue,ORDERLY_DELAY );//portMAX_DELAY
//        state = eTaskGetState(orderlyHandling);
        //X_axis_int
        //X_axis_int_fin
        //SignalUSBbufferReady (1<<2) // Получена команда по каналу USB.
        //ot_sgQueueEmpty     (1<<3) //выполнена ОБработка всех сегментов и новых сегментов нет.
        //ot_sgTest

        if(ulNotifiedValue & SignalUSBbufferReady){
            msegment = (struct ComDataReq_t *)cmdBuffer_usb;
            /* Потсупила новая команда.
             * Parse command;
             * TakeSemaphore_rcvd
             * send status
             */

            switch(msegment->command.order)
            {
            case eoSegment: // new Command&data received.
                //                sema = xSemaphoreTake(rcvd_semaphore_handler,5);
                //                if(sema == pdPASS){
                //                    if(MEMF_GetNumFreeBlocks() != 0)
                //                        if(msegment->instrument1_parameter.head.linenumber >
                //                                getHeadLineNumber())
                //                        {
                //                            if(rcvd_SegmentFlag == 0){
                //                                memcpy(segmentBuffer,&msegment->instrument1_parameter,sizeof(struct sSegment));
                //                                rcvd_SegmentFlag = 1;
                //                            }
                //                        }
                //                    xSemaphoreGive(rcvd_semaphore_handler);
                //                    xTaskNotify(sectorHandling,sg_segmentRecieved,eSetBits);
                //                    xStatus = xQueueSend(segmentQueueHandler,&msegment->instrument1_parameter,0);
                ss = (uint8_t*)MEMF_Alloc();
                memcpy(ss, &msegment->payload.instrument1_parameter, sizeof(struct sSegment));
                if(pMs_State->instrumrnt1 == eIns1_stoped){
                    pblockSegment(plan_get_current_block());
                    //                        start_t1(0); // debug
                    NoOperation;
                }
                else{
                    NoOperation;
                }

                sendStatus();

                break;

            case eoProfile:
                NoOperation; // TODO hotend parameters

                sendStatus();

                break;
            }

            NoOperation;
        }

        if(ulNotifiedValue & X_axis_int){
//            axisX_rateHandler();
            taskcounter++;
//            ms_nextSector();
        }

        if(ulNotifiedValue & X_axis_int_fin)
        {
//            ms_finBlock = continueBlock;
//            start_t1(0);
            NoOperation;    // TODO Kalibrovka
        }

        if(ot_sgQueueEmpty & ulNotifiedValue){
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            NoOperation;
        }

        if(ot_sgTest & ulNotifiedValue){
            // start test sequence.
            initBlock();    // initialize sectors.
            initStepper(N_AXIS);
            ms_finBlock = continueBlock;
//            start_t1(0); // debug

        }

        if(ender_xmin_test & ulNotifiedValue){
            NoOperation;
        }
//        if(ulNotifiedValue & 0x02){
//            rgb_disable();
//            NoOperation;
//        }
        if(ret == pdFALSE){
            // Отправка статуса устройства.
            if(pMs_State->instrumrnt1 == eIns1_stoped){
                ms_nextSector();    // Индикация
            }else{
                HWREGBITB(&be_portf3,3) = ~HWREGBITB(&be_portf3,3);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, be_portf3);
            }
        }

    } // end for(;;);

}


uint32_t createtask_orderly(void)
{
    //orderlyTask()

    //
    // Create the Orderly task.
    //
    if(xTaskCreate(orderly_routine, (const portCHAR *)"Orderly", ORDERLYTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_ORDERLY_TASK, &orderlyHandling) != pdTRUE)
    {
        return(1);
    }else{

//        tskTaskControlBlock* tt = prvGetTCBFromHandle(orderlyHandling);
        init_Status();
        return (0);
    }



}
