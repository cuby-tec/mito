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

#define HOTENDTASKSTACKSIZE 180//80//64

#define HOTEND_DELAY        500

#define EXAMPLE2

// Control loop gains
#ifdef EXAMPLE1
#define KD  0.120//0.1//0.013
#define KP  0.8//0.75
#define KI  1.0 - KP - KD//0.15
#define PIDMAXVALUE HOTEND_D10_PERIOD//4294967295
#define PIDMINVALUE 0
#define SAMPLETIME  HOTEND_DELAY
#define SETPOINT    40
#endif
#ifdef EXAMPLE2
#define KD  12.5//0.1//0.013
#define KP  0.6//0.75
//#define KI  1.0 - KP - KD//0.15
#define KI  0.1
#define PIDMAXVALUE HOTEND_D10_PERIOD//4294967295
#define PIDMINVALUE 0
#define SAMPLETIME  HOTEND_DELAY
#define SETPOINT    40
#endif
//-------------- vars
TaskHandle_t hotendHadling;
#ifdef StackType_t
StackType_t hotStack[HOTENDTASKSTACKSIZE];

StaticTask_t hotTaskBuffer;
#endif
static float_t temperature;
static float_t current_temperature;
static float_t current_resistance;
static uint32_t adc;


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
//    return current_resistance;
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
    uint32_t value;
    if(out>0){
        value = out *800;//500;
//        value *= 500;
        if(value > HOTEND_D10_PERIOD)
            value = HOTEND_D10_PERIOD -10;

     }else{
        value = 2;
    }

    setTIMER_HOTEND(value);

    NoOperation;
}

//static float temperature;



static uint32_t ticks = 100;

uint32_t tick_get()
{
//  result += 100;
    return ticks;
}


static  uint32_t counter = 0;

static void hotend_routine(void* pvParameters)
{
    static BaseType_t ret;
    static uint32_t ulNotifiedValue;

    for(;;){

        ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, HOTEND_DELAY);

        adc = get_hotend_adc();
        current_resistance = ((float)4.7)/(((float)4096.0/((float)adc+0.5) - 1));

            current_temperature = get_temperature(current_resistance);
/*
        if(!isnan(current_temperature )){
            counter ++;
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
*/
    } // end of for(;;)
}
#ifdef StackType_t
#define IDLE_TASK_SIZE 200

#if configSUPPORT_STATIC_ALLOCATION
    /* static memory allocation for the IDLE task */
    static StaticTask_t xIdleTaskTCBBuffer;
    static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];//IDLE_TASK_SIZE configMINIMAL_STACK_SIZE

    void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
        *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
        *ppxIdleTaskStackBuffer = &xIdleStack[0];
        *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;//IDLE_TASK_SIZE
    }
#endif
#endif


uint32_t createtask_hotend(void)
{

    //
    // Create the Orderly task.
    //
    TaskHandle_t t;
    void* pvParameters = NULL;

#ifdef StackType_t
    const portCHAR* pcName = (const portCHAR *)"Hotend";
    hotendHadling = xTaskCreateStatic(hotend_routine, pcName, HOTENDTASKSTACKSIZE, pvParameters, tskIDLE_PRIORITY + PRIORITY_HOTEND_TASK, hotStack, &hotTaskBuffer);

    //    if(xTaskCreate(hotend_routine, (const portCHAR *)"Hotend", HOTENDTASKSTACKSIZE, NULL,
    //                   tskIDLE_PRIORITY + PRIORITY_HOTEND_TASK, &hotendHadling) != pdTRUE)
    if(hotendHadling == pdFALSE)
    {
        return(1);// failure
    }else{
        init_hotend();
        return (0);
    }

#else
    if(xTaskCreate(hotend_routine, (const portCHAR *)"Hotend", HOTENDTASKSTACKSIZE, NULL,
                       tskIDLE_PRIORITY + PRIORITY_HOTEND_TASK, &hotendHadling) != pdTRUE)
    {
        return(1);// failure
    }else{
        init_hotend();
        return (0);
    }

#endif


}


