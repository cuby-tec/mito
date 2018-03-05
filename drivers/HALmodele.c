/*
 * HALmodele.c
 *
 *  Created on: 28 февр. 2018 г.
 *      Author: walery
 */

//--------------

#include <FreeRTOS.h>
#include <task.h>

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "HALmodele.h"

#include "msmotor/ms_model.h"
#include "switch_task.h"
#include "orderlyTask.h"

#include "kalibrovka/ms_kalibrovka.h"


//#include "msmotor/msport.h"


//------------- defs

//-------------- vars


//-------------- function

int32_t getEnders()
{
    int32_t result;
    result = GPIOPinRead(ENDER_BASE, ENDER_Z_MIN|ENDER_Z_MAX
                         |ENDER_Y_MIN|ENDER_Y_MAX|ENDER_X_MIN|ENDER_X_MAX);

    return result;
}

int32_t getXminEnder()
{
    int32_t result;//,a = 0;
    result = GPIOPinRead(ENDER_BASE, ENDER_X_MIN);
//    HWREGBITW(&a, ENDER_X_MIN) ^=1 ;
    return (result&ENDER_X_MIN?true:false); //result;
}

void set_Xmin_IntType_rising()
{
    uint32_t zender = getEnders();

    if(zender & ENDER_X_MIN){
        while(1)
        {
            NoOperation;
        }
    }

    GPIOIntDisable(ENDER_BASE, ENDER_X_MIN);
    // sets detection to edge and trigger to rising
    GPIOIntTypeSet(ENDER_BASE, ENDER_X_MIN, GPIO_RISING_EDGE);
//    GPIOIntTypeSet(ENDER_BASE, ENDER_X_MIN, GPIO_HIGH_LEVEL);
    GPIOIntClear(ENDER_BASE, ENDER_X_MIN);
    GPIOIntEnable(ENDER_BASE, ENDER_X_MIN);
}


void set_Xmin_IntType_falling()
{

    GPIOIntDisable(ENDER_BASE, ENDER_X_MIN);
    // sets detection to edge and trigger to rising
    GPIOIntTypeSet(ENDER_BASE, ENDER_X_MIN, GPIO_FALLING_EDGE);
    GPIOIntClear(ENDER_BASE, ENDER_X_MIN);
    GPIOIntEnable(ENDER_BASE, ENDER_X_MIN);

}

/**
 * Установка типа прерывания для концевого выключателя.
 */
void set_EnderEdge(enum ender_edge edge)
{
    switch(edge){
    case kl_xminrise:
        GPIOIntDisable(ENDER_BASE, ENDER_X_MIN);
        // sets detection to edge and trigger to rising
        GPIOIntTypeSet(ENDER_BASE, ENDER_X_MIN, GPIO_RISING_EDGE);
    //    GPIOIntTypeSet(ENDER_BASE, ENDER_X_MIN, GPIO_HIGH_LEVEL);
        GPIOIntClear(ENDER_BASE, ENDER_X_MIN);
        GPIOIntEnable(ENDER_BASE, ENDER_X_MIN);
        break;

    case kl_xminfall:
        GPIOIntDisable(ENDER_BASE, ENDER_X_MIN);
        // sets detection to edge and trigger to rising
        GPIOIntTypeSet(ENDER_BASE, ENDER_X_MIN, GPIO_FALLING_EDGE);
        GPIOIntClear(ENDER_BASE, ENDER_X_MIN);
        GPIOIntEnable(ENDER_BASE, ENDER_X_MIN);
        break;

    case kl_xmaxrise:
        GPIOIntDisable(ENDER_BASE, ENDER_X_MAX);
        // sets detection to edge and trigger to rising
        GPIOIntTypeSet(ENDER_BASE, ENDER_X_MAX, GPIO_RISING_EDGE);
        GPIOIntClear(ENDER_BASE, ENDER_X_MAX);
        GPIOIntEnable(ENDER_BASE, ENDER_X_MAX);
        break;

    case kl_xmax_fall:
        GPIOIntDisable(ENDER_BASE, ENDER_X_MAX);
        // sets detection to edge and trigger to rising
        GPIOIntTypeSet(ENDER_BASE, ENDER_X_MAX, GPIO_FALLING_EDGE);
        GPIOIntClear(ENDER_BASE, ENDER_X_MAX);
        GPIOIntEnable(ENDER_BASE, ENDER_X_MAX);
        break;


    }// end switch(edge)
}

void set_DisableIntEnders(void)
{
    GPIOIntDisable(ENDER_BASE, ENDER_X_MIN | ENDER_X_MAX);
}

//---------------------------- interrupt handler ----------
void PortEnderIntHandler()
{
    uint32_t intstatus;
    GPIOIntDisable(ENDER_BASE, ENDER_X_MIN);
//    stop_xkalibrovka();
    // If bMasked is set as true, then the masked interrupt status is returned;
    intstatus = GPIOIntStatus(ENDER_BASE, false);
    if( ENDER_X_MIN & intstatus){
        GPIOIntClear(ENDER_BASE, ENDER_X_MIN);
        stop_xkalibrovka(X_AXIS);
        xTaskNotifyFromISR(switchTaskHandle,ENDER_XMIN_HANDLE,eSetBits,NULL);
//        orderlyHandling
//        xTaskNotifyFromISR(orderlyHandling,ender_xmin_test,eSetBits,NULL);

    }else if( ENDER_X_MAX & intstatus){
        GPIOIntClear(ENDER_BASE, ENDER_X_MAX);
        stop_xkalibrovka(X_AXIS);
        xTaskNotifyFromISR(switchTaskHandle,ENDER_XMAX_HANDLE,eSetBits,NULL);
    }

}


