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

//---------------------------- interrupt handler ----------
void PortEnderIntHandler()
{
    uint32_t intstatus;
    GPIOIntDisable(ENDER_BASE, ENDER_X_MIN);
//    stop_xkalibrovka();
    // If bMasked is set as true, then the masked interrupt status is returned;
    intstatus = GPIOIntStatus(ENDER_BASE, false);
    if(intstatus & ENDER_X_MIN){
        GPIOIntClear(ENDER_BASE, ENDER_X_MIN);
        stop_xkalibrovka();
        xTaskNotifyFromISR(switchTaskHandle,ENDER_XMIN_HANDLE,eSetBits,NULL);
        interrupt_counter++;
//        orderlyHandling
//        xTaskNotifyFromISR(orderlyHandling,ender_xmin_test,eSetBits,NULL);

    }else{
        NoOperation;
    }

}


