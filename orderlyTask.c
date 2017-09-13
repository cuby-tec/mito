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
#include "inc/hw_types.h"
//#include "drivers/rgb.h"
//#include "drivers/sysctl.h"
//#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

#include "orderlyTask.h"
#include "msmotor/mempool.h"
#include "msmotor/msport.h"
//#include "msmotor/ms_init.h"
#include "memf/mSegmentQuee.h"
#include "msmotor/ms_model.h"
#include "exchange/ComDataReq_t.h"

#include "priorities.h"
#include <limits.h>

#include "exchange/status.h"



//------------- DEFS

#define ORDERLYTASKSTACKSIZE   64//128//640//576 //128 //192 640 //

//---------  vars

//Task_Struct taskOrderlyStruct;
//Char taskOrderlyStack[TASKSTACKSIZE];

//static UInt32 time1,time2, time3 = 0;
//static Types_FreqHzf hz;


TaskHandle_t orderlyHandling;

uint32_t taskcounter = 0;


static struct ComDataReq_t* msegment;

//--------- function



//static uint32_t cnt_delay;
//static uint32_t delay_max;
void orderly_routine(void* pvParameters ){
    uint32_t ulNotifiedValue;
    volatile eTaskState state;

//    testPrepare();
//    initBlock();

    initStepper(N_AXIS);
    ms_finBlock = exitBlock;
    start_t1(0);
    for( ;; ){
//portMAX_DELAY
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
//        state = eTaskGetState(orderlyHandling);


        if(ulNotifiedValue & SignalUSBbufferReady){
            msegment = (struct ComDataReq_t *)cmdBuffer_usb;
            NoOperation;
        }

        if(ulNotifiedValue & X_axis_int){
//            axisX_rateHandler();
            taskcounter++;
//            ms_nextSector();
        }

        if(ulNotifiedValue & X_axis_int_fin)
        {
            ms_finBlock = continueBlock;
            start_t1(0);
        }

//        if(ulNotifiedValue & 0x02){
//            rgb_disable();
//            NoOperation;
//        }

    } // end for(;;);

}


uint32_t createtask_orderly(void){
    //TODO orderlyTask()
/*

     Construct BIOS Objects
    Task_Params taskParams;
     Construct writer/reader Task threads
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 10;
    taskParams.stack = &taskOrderlyStack;


    Task_construct(&taskOrderlyStruct, (Task_FuncPtr)orderly_routine, &taskParams, NULL);

    msInit(0);
*/
    //
    // Create the Orderly task.
    //
    if(xTaskCreate(orderly_routine, (const portCHAR *)"Orderly", ORDERLYTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_ORDERLY_TASK, &orderlyHandling) != pdTRUE)
    {
        return(1);
    }else{
        return (0);
    }



}
