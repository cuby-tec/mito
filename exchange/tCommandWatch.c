/*
 * tCommandWatch.c
 * Company: CUBY,Ltd
 *  Created on: 18 мая 2018 г.
 *      Author: walery
 */



#include "tCommandWatch.h"

#include "priorities.h"

#include "drivers/usbmodule.h"
#include "msmotor/mempool.h"
#include "orderlyTask.h"


//----------- defs

#define COMMANDWATCHSTACKSIZE   64//128//40
/*
 *     xHigherPriorityTaskWoken = pdFALSE;

    if(xQueueSendFromISR(orderlyQueue,&cmdBuffer_usb, &xHigherPriorityTaskWoken)) //;//&xHigherPriorityTaskWoken
    {
        tmpcounter++;
    }else{
        tmpcounter--;
    }
 */
//------------- vars

//static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//------------- function

static void
commandWatch_routine()
{
    for(;;){

        vTaskDelay(10);
        if(commandFlag)
        {
            if(xQueueSend(orderlyQueue,&cmdBuffer_usb,portMAX_DELAY)==pdPASS)
            {
                commandFlag = false;

            }
            NoOperation;

        }

    }// end of for(;;)
}


/*
 * Create task
 */
int
createCommandWatch()
{
   if( xTaskCreate(commandWatch_routine, "CommandWatch", COMMANDWATCHSTACKSIZE, NULL,
                PRIORITY_commandWatch, NULL) == pdFALSE)
   {
       return (1);
   }
   else{
       return (0);
   }

}
