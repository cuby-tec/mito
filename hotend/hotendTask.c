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
#include "pid/PID.h"

//------------- defs

#define HOTENDTASKSTACKSIZE 80//64

#define HOTEND_DELAY        500

// Control loop gains
#define KD  0.120//0.1//0.013
#define KP  0.8//0.75
#define KI  1.0 - KP - KD//0.15
#define PIDMAXVALUE 4294967295
#define PIDMINVALUE 0
#define SAMPLETIME  10
#define SETPOINT    40

//-------------- vars
TaskHandle_t hotendHadling;


static float_t temperature;
static float_t current_temperature;


// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata;
_pid_t pid;


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

    /**
     * init PID regulator
     */

    setTargetHotendTemperature(SETPOINT);

    // Prepare PID controller for operation
    pid = pid_create(&ctrldata, getTargetHotendTemperature(), KP, KI, KD, SAMPLETIME);
    pid_limits(pid, PIDMINVALUE, PIDMAXVALUE);
    // Allow PID to compute and change output
    pid_auto(pid);


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

/**
 * Управление таймером ХОТЕНДА
 */
static
process_output(float out){
    if(out>0)
        setTIMER_HOTEND(out);
    NoOperation;
}

//static float temperature;

static uint32_t adc;


static uint32_t ticks = 100;

uint32_t tick_get()
{
//  result += 100;
    return ticks;
}




static void hotend_routine(void* pvParameters)
{
    static BaseType_t ret;
    static uint32_t ulNotifiedValue;

    for(;;){

        ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, HOTEND_DELAY);

        adc = get_hotend_adc();
        current_temperature = ((float)4.7)/(((float)4096.0/((float)adc+0.5) - 1));

            current_temperature = get_temperature(current_temperature);

        if(!isnan(current_temperature )){

            // Check if need to compute PID
            if (pid_need_compute(pid)) {
                // Read process feedback
                //            pid->input = process_input();
                pid->input = current_temperature;
                // Compute new PID output value
                pid_compute(pid);
                //Change actuator value
                process_output(pid->output);
                //          if(i == TICK/2){
                //              pid->setpoint = 150.0;
                //          }
                ticks += SAMPLETIME;
            } // end of if
        }

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


