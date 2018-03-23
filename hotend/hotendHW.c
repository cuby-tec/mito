/*
 * hotendHW.c
 *
 *  Created on: 23 мар. 2018 г.
 *      Author: walery
 */

//--------------


#include "hotendHW.h"

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "msmotor/msport.h"
#include "inc/typedefs.h"

//------------- defs

//-------------- vars


//-------------- function
/**
 *     SysCtlPeripheralEnable(TIMER_X_AXIS_PERIPH);
 *     //  Timer X initializing. ======================
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBMR) = TIMER_TBMR_TBMR_PERIOD|TIMER_TBMR_TBAMS|TIMER_TBMR_TBPWMIE;
//    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TAMR) |= TIMER_TBMR_TAMIE;
//    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TAMR) |= TIMER_TBMR_TAPLO;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBMR) |= TIMER_TBMR_TBILD;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBMR) |= TIMER_TBMR_TBMRSU;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_IMR)  |=  TIMER_IMR_CBEIM;// TIMER_IMR_TAMIM; // | TIMER_IMR_TATOIM;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_Y_AXIS  + TIMER_O_TAPR) = 0;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBSTALL;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEVENT_NEG;     //TIMER_CTL_TAEVENT_POS ;
//    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEVENT_POS;
//    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBPWML; //  TABPWML inverted
    HWREG(TIMER_BASE_Y_AXIS  + TIMER_O_TBMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_Y_AXIS  + TIMER_O_TBPMR) = 0;

    TimerIntClear(TIMER_BASE_Y_AXIS, TIMER_CAPB_EVENT);
    IntEnable(INT_TIMER1B); // NVIC register setup.
    IntPrioritySet(INT_TIMER_Y, INT_TIMER_Y_PRIORITY);

 *     //        TimerEnable(TIMER_BASE_X_AXIS, TIMER_X);
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN;
 *
 */
void initHotendHW(void)
{
    // WT0CCP1 @ PC5


    SysCtlPeripheralEnable(TIMER_HOTEND_PERIPH);

    HWREG(TIMER_BASE_HOTEND + TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //
    HWREG(TIMER_BASE_HOTEND + TIMER_O_CFG) = TIMER_CFG_16_BIT; // set 32 bit mode
    HWREG(TIMER_BASE_HOTEND + TIMER_O_TBMR) = TIMER_TBMR_TBMR_PERIOD|TIMER_TBMR_TBAMS|TIMER_TBMR_TBPWMIE;
    HWREG(TIMER_BASE_HOTEND + TIMER_O_TBMR) |= TIMER_TBMR_TBILD;
    HWREG(TIMER_BASE_HOTEND + TIMER_O_TBMR) |= TIMER_TBMR_TBMRSU;
    HWREG(TIMER_BASE_HOTEND + TIMER_O_IMR)  |=  TIMER_IMR_CBEIM;// TIMER_IMR_TAMIM; // | TIMER_IMR_TATOIM;
    HWREG(TIMER_BASE_HOTEND + TIMER_O_TBILR) = 0xc355;    //720895
    HWREG(TIMER_BASE_HOTEND  + TIMER_O_TAPR) = 0; // Prescale

    HWREG(TIMER_BASE_HOTEND + TIMER_O_CTL) |= TIMER_CTL_TBSTALL;
    HWREG(TIMER_BASE_HOTEND + TIMER_O_CTL) |= TIMER_CTL_TBEVENT_NEG;     //TIMER_CTL_TAEVENT_POS ;
    HWREG(TIMER_BASE_HOTEND  + TIMER_O_TBMATCHR) = 0x0fff;
    HWREG(TIMER_BASE_HOTEND  + TIMER_O_TBPMR) = 0; // GPTM TimerB Prescale Match
    TimerIntClear(TIMER_BASE_HOTEND, TIMER_CAPB_EVENT);
    IntEnable(INT_HOTEND_TIMER); // NVIC register setup.
    IntPrioritySet(INT_HOTEND_TIMER, INT_HOTEND_TIMER_PRIORITY);
    //        TimerEnable(TIMER_BASE_X_AXIS, TIMER_X);
        HWREG(TIMER_BASE_HOTEND + TIMER_O_CTL) |= TIMER_CTL_TBEN;

NoOperation;
}

void intHotendHandler(void)
{
    TimerIntClear(TIMER_BASE_HOTEND, TIMER_CAPB_EVENT);
    NoOperation;
}

