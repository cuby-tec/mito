/*
 * orderlyTask.c
 *
 *  Created on: 18 июн. 2017 г.
 *      Author: walery
 */

#include "orderlyTask.h"
//#include <xdc/runtime/Timestamp.h>

//#include <rgb.h>
//#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
//#include "drivers/rgb.h"
//#include "drivers/sysctl.h"
//#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"


#include "msmotor/ms_init.h"
#include "msmotor/ms_model.h"


#include "priorities.h"
#include <limits.h>
//------------- DEFS

#define ORDERLYTASKSTACKSIZE   192

//---------  vars

//Task_Struct taskOrderlyStruct;
//Char taskOrderlyStack[TASKSTACKSIZE];

//static UInt32 time1,time2, time3 = 0;
//static Types_FreqHzf hz;


TaskHandle_t orderlyHandling;


//--------- function


/*
Void orderly_routine(UArg arg0, UArg arg1){



  //
    // Enable and wait for the port to be ready for access
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

//    msInit(0);



    for (;;) {
        //
        // Turn on the LED
        //
//        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED);
        time1 = Clock_getTicks();
        System_printf("Tick in system. cnt_int: %lu\n",cnt_int);
        System_flush();
        Task_sleep(1000);

        //
        // Turn on the LED
        //
//        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, BLUE_LED);
        Task_sleep(1000);
        time2 = Clock_getTicks();
        time3++;
        System_printf("Delta time is :%lu ; now: %lu \n",(ULong)(time2 - time1),time3);


    } // end of for(;;);
}

*/
static uint32_t cnt_delay;
static uint32_t delay_max;
void orderly_routine(void* pvParameters ){
    uint32_t ulNotifiedValue;

    for( ;; ){

        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);

        if(ulNotifiedValue & 0x01){
//            prvProcessBit0Event();
            cnt_delay = TimerValueGet(WTIMER5_BASE, TIMER_B);
            cnt_delay -= ms_delay.counter;
            if(delay_max<cnt_delay)
                delay_max = cnt_delay;
            rgb_enable();
            NoOperation;
        }
        if(ulNotifiedValue & 0x02){
            rgb_disable();
            NoOperation;
        }

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
