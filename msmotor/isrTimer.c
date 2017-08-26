/*
 * isrTimer_X.c
 *
 *  Created on: 1 июл. 2017 г.
 *      Author: walery
 */

//--------------



#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#ifdef TARGET_IS_TM4C123_RA1
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#endif

#include "msport.h"
#include "msmotor/isrTimer.h"
#include "mempool.h"

//------------- defs

//-------------- vars
static uint32_t multy = 1;

//-------------- function

#ifdef commit_13
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
#endif

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
// X axis.
//
//*****************************************************************************
void
Timer1IntHandler(void)
{
    //    uint32_t cOne = 1, cTwo;

    uint32_t timerValue_X;
    //    uint32_t timerValueMatch;

    TimerIntClear(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);
    //    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);

    //
    // Toggle the flag for the second timer.
    //
    //    HWREGBITW(&g_ui32Flags, 1) ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    sts.counter++;

    if(sts.counter>sts.point){
        HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    }else{
        timerValue_X = sts.rate*multy;
        //           timerValueMatch = timerValue - 128;// timerValue/8;
        //           dump[ sts.counter_y%60] = sts.rate_y;

        TimerLoadSet(TIMER_BASE_X_AXIS,TIMER_X,timerValue_X);
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = sts.rate_y*multy;
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = (uint16_t)(timerValue&0x0000FFFF);
        timerValue_X >>=16;
        TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_X, (uint16_t)(timerValue_X&0x000000FF));
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAPR) = (uint16_t)(uint16_t)(timerValue&0x000000FF);

    }
    axis_flags |= X_FLAG;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.
    //           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    return;
}

void TimerEIntHandler(void)
{
    uint32_t timerValue_E;

    // Clear the timer interrupt.
    TimerIntClear(TIMER_BASE_E_AXIS, TIMER_CAPB_EVENT);

    // Toggle the flag for the first timer.
    //    HWREGBITW(&g_ui32Flags, 0) ^= 1;
    //    GPIOPinWrite(DIRECTION_PORT, DIRECTION_Y, DIRECTION_Y);

    sts_e.counter++;

    if(sts_e.counter>sts_e.point){
        HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
    }else{
        timerValue_E = sts_e.rate*multy;
        //           timerValueMatch = timerValue_Z - 128;// timerValue_Z/8;
        //           dump[ sts_e.counter%60] = sts.rate_z;

        TimerLoadSet(TIMER_BASE_E_AXIS, TIMER_E, timerValue_E);
        timerValue_E >>=16;
        TimerPrescaleSet(TIMER_BASE_E_AXIS, TIMER_E, (uint16_t)(timerValue_E&0x000000FF));
    }
    axis_flags |= E_FLAG;
    //    GPIOPinWrite(DIRECTION_PORT, DIRECTION_Y, ~DIRECTION_Y);
    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.

    return;

}

//  Axis Y handler
void TimerYIntHandler(void)
{
    //    char cOne, cTwo;
    uint32_t timerValue_Y;

    // Clear the timer interrupt.
    TimerIntClear(TIMER_BASE_Y_AXIS, TIMER_CAPB_EVENT);

    //
    // Toggle the flag for the first timer.
    //
    //    HWREGBITW(&g_ui32Flags, 0) ^= 1;
    //    GPIOPinWrite(DIRECTION_PORT, DIRECTION_Y, DIRECTION_Y);

    sts_y.counter++;

    if(sts_y.counter>sts_y.point){
        HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
    }else{
        timerValue_Y = sts_y.rate*multy;
        //           timerValueMatch = timerValue - 128;// timerValue/8;
        //           dump[ sts.counter%60] = sts.rate_y;

        TimerLoadSet(TIMER_BASE_Y_AXIS, TIMER_Y, timerValue_Y);
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = sts.rate_y*multy;
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = (uint16_t)(timerValue&0x0000FFFF);
        timerValue_Y >>=16;
        TimerPrescaleSet(TIMER_BASE_Y_AXIS, TIMER_Y, (uint16_t)(timerValue_Y&0x000000FF));
    }
    axis_flags |= Y_FLAG;
    //    GPIOPinWrite(DIRECTION_PORT, DIRECTION_Y, ~DIRECTION_Y);
    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.
    //           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    return;

    //
    // Use the flags to Toggle the LED for this timer
    //
    //    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, g_ui32Flags << 1);

}

// Axis Z handler -------------------------------
void TimerZIntHandler(void)
{
    uint32_t timerValue_Z;

    // Clear the timer interrupt.
    TimerIntClear(TIMER_BASE_Z_AXIS, TIMER_CAPA_EVENT);

    // Toggle the flag for the first timer.
    //    HWREGBITW(&g_ui32Flags, 0) ^= 1;
    //    GPIOPinWrite(DIRECTION_PORT, DIRECTION_Y, DIRECTION_Y);

    sts_z.counter++;

    if(sts_z.counter>sts_z.point){
        HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    }else{
        timerValue_Z = sts_z.rate*multy;
        //           timerValueMatch = timerValue_Z - 128;// timerValue_Z/8;
        //           dump[ sts_z.counter%60] = sts.rate_z;

        TimerLoadSet(TIMER_BASE_Z_AXIS, TIMER_Z, timerValue_Z);
        timerValue_Z >>=16;
        TimerPrescaleSet(TIMER_BASE_Z_AXIS, TIMER_Z, (uint16_t)(timerValue_Z&0x000000FF));
    }
    axis_flags |= Z_FLAG;
    //    GPIOPinWrite(DIRECTION_PORT, DIRECTION_Y, ~DIRECTION_Y);
    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.

    return;
}

