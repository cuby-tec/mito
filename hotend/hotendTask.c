/*
 * hotendTask.c
 *
 *  Created on: 22 мар. 2018 г.
 *      Author: walery
 */

//--------------




#include <stdbool.h>
#include "hotendTask.h"

#include "hotendHW.h"

#include "priorities.h"
#include <limits.h>
#include "inc/typedefs.h"
//------------- defs

#define HOTENDTASKSTACKSIZE 64

#define HOTEND_DELAY        500

//-------------- vars
TaskHandle_t hotendHadling;


static float_t temperature;
static float_t current_temperature;

//-------------- function


void setTargetHotendTemperature(float_t temp)
{
    NoOperation;
    temperature = temp;
}

float_t getTargetHotendTemperature(void)
{
    return temperature;
}

void setCurrentHotendTemperature(float_t temp)
{
    NoOperation;
    current_temperature = temp;
}

float_t getCurrentHotendTemperature(void)
{
    return current_temperature;
}


static void init_heater(void)
{
// WT0CCP1 @ PC5
    initHotendHW();

}

void init_hotend(void)
{
    init_heater();
}

// Запустить Hotend инструмент.
void start_hotende(void)
{
    //TODO start hotend.
    NoOperation;
}

void stop_hotend(void)
{
    // TODO stop hotend
    NoOperation;
}

static uint32_t adc;

static void hotend_routine(void* pvParameters)
{
    static BaseType_t ret;
    static uint32_t ulNotifiedValue;

    for(;;){

        ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, HOTEND_DELAY);


        adc = get_hotend_adc();
        NoOperation;

    } // end of for(;;)
}



uint32_t createtask_hotend(void)
{
    //
    // Create the Orderly task.
    //
    if(xTaskCreate(hotend_routine, (const portCHAR *)"Hotend", HOTENDTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_HOTEND_TASK, &hotendHadling) != pdTRUE)
    {
        return(1);// failure
    }else{
        init_hotend();
        return (0);
    }


}


