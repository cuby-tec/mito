/*
 * isrTimer_X.c
 *
 *  Created on: 1 июл. 2017 г.
 *      Author: walery
 */

//--------------


#include "isrTimer_X.h"
#include "msport.h"

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#ifdef TARGET_IS_TM4C123_RA1
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#endif


//------------- defs

//-------------- vars


//-------------- function


void Timer_X_isr(void){
    static uint32_t cnt_x;
    TimerIntClear(TIMER_BASE_X_AXIS,TIMER_CAPA_EVENT);//TIMER_TIMB_TIMEOUT CAEIM
    cnt_x++;
    if(cnt_x & 1){
        GPIOPinWrite(GPIO_PORTF_BASE, RED_GPIO_PIN, RED_GPIO_PIN);
//        xTaskNotifyFromISR(orderlyHandling,0x01,eSetBits,NULL);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE, RED_GPIO_PIN, ~RED_GPIO_PIN);
//        xTaskNotifyFromISR(orderlyHandling,0x02,eSetBits,NULL);
    }
}


