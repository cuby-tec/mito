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

#include "thermistor/thermo.h"

//------------- defs

#define HOTENDTASKSTACKSIZE 80//64

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


static float temperature;

static uint32_t adc;


static int32_t counter_mean;
static uint32_t adc_1,adc_2;    // current and before value;
static uint32_t adc_mean = 0;

static void hotend_routine(void* pvParameters)
{
    static BaseType_t ret;
    static uint32_t ulNotifiedValue;

    for(;;){

        ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, HOTEND_DELAY);


        adc = get_hotend_adc();
        if(counter_mean == 0){
//            adc_1 = adc;
            adc_2 = adc;
            adc_mean +=adc;
            counter_mean ++;
        }else if(counter_mean <20){
//            adc_1 = adc;
//            adc_mean -= adc_2;
            adc_mean += adc;
            adc_2 = adc;
            counter_mean ++;
        }else{
//            adc_1 = adc;
            adc_mean -= adc_2;
            adc_mean += adc;
            adc_2 = adc;
        }

//        adc = ((float)adc_mean+0.5)/counter_mean;

        temperature = ((float)4.7)/(((float)4096/(((float)adc_mean+0.5)/counter_mean) -1));

        temperature = get_temperature(temperature);

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


