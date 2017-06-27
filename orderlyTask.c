/*
 * orderlyTask.c
 *
 *  Created on: 18 июн. 2017 г.
 *      Author: walery
 */

#include "orderlyTask.h"
//#include <xdc/runtime/Timestamp.h>

//#include <rgb.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
//#include "drivers/rgb.h"
//#include "drivers/sysctl.h"
//#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"


#include "msmotor/ms_init.h"
#include "msmotor/ms_model.h"

//------------- DEFS

#define TASKSTACKSIZE   512

//---------  vars

//Task_Struct taskOrderlyStruct;
//Char taskOrderlyStack[TASKSTACKSIZE];

//static UInt32 time1,time2, time3 = 0;
//static Types_FreqHzf hz;

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

void createtask_orderly(void){
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


}
