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
#include "exchange/ComDataReq_t.h"

//----------- defs

#define COMMANDWATCHSTACKSIZE   128//40
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
/**
 * Определение типа команды.
 */
static uint16_t
controlCommandParcer()
{
    //orderlyQueue,&cmdBuffer_usb
    struct ComDataReq_t * req = (struct ComDataReq_t *)&cmdBuffer_usb;
    return (req->command.order);
}



static void
commandWatch_routine(void * pvParameters)
{
    static struct ComDataReq_t* request;

    for(;;){

        vTaskDelay(10);
        if(commandFlag)
        {
            switch(controlCommandParcer())
            {
            case eoState:
                sendStatus();
                NoOperation;
                break;
            case eoProfile:
                if(xQueueSend(orderlyQueue,&cmdBuffer_usb,portMAX_DELAY)!=pdPASS)
                {
                    // Exception
                    while(1){
                        NoOperation;
                    }
                }
                NoOperation;
                break;

            case eoSegment:
                //QueueHandle_t segmentQueue;
                request =  (struct ComDataReq_t *)&cmdBuffer_usb;
                if(xQueueSend(segmentQueue,&request->payload.instrument1_parameter,NULL) == pdFAIL)
                {
                    sendStatus();
                }else{
                    sendStatus();
                }

                UBaseType_t size = uxQueueSpacesAvailable( segmentQueue );
                size++;

                break;

            default:
                if(xQueueSend(orderlyQueue,&cmdBuffer_usb,portMAX_DELAY)!=pdPASS)
                {
                    // Exception
                    while(1){
                        NoOperation;
                    }
                }
                NoOperation;
                break;

            }
            commandFlag = false;
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
       if(!createCommandPool())
       {
           while(1)
           {
               NoOperation;
           }
       }
       return (0);
   }

}
