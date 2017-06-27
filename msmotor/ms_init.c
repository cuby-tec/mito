/*
 * ms_init.c
 *
 *  Created on: 19 июн. 2017 г.
 *      Author: walery
 */


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#ifdef TARGET_IS_TM4C123_RA1
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#endif
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"


#include "drivers/pinout.h"

#include "ms_init.h"
#include "ms_model.h"
#include "msport.h"
//#include "packages/ti/sysbios/hal/Hwi.h"
//#include "xdc/runtime/Error.h"
//#include "xdc/runtime/System.h"
//#include "ti/sysbios/hal/Timer.h"
//------------- defs

//#define TIVA
#define USE_HW

//-------------- function


//*****************************************************************************
//
//! Initializes the Timer and GPIO functionality associated with the RGB LED
//!
//! \param ui32Enable enables RGB immediately if set.
//!
//! This function must be called during application initialization to
//! configure the GPIO pins to which the LEDs are attached.  It enables
//! the port used by the LEDs and configures each color's Timer. It optionally
//! enables the RGB LED by configuring the GPIO pins and starting the timers.
//!
//! \return None.
//
//*****************************************************************************
void msInit(uint32_t ui32Enable){
#ifdef HWI
    Hwi_Handle myHwi;
    Error_Block eb;

    Error_init(&eb);

    myHwi = Hwi_create(5, axisX_intrrupt_handler,NULL,&eb);
    if (myHwi == NULL) {
    System_abort("Hwi create failed");
    }


    Timer_Params timerParams;
    Timer_Handle myTimer;
    Error_Block eb1;

    Error_init(&eb1);

    Timer_Params_init(&timerParams);
    timerParams.period = 10;
    timerParams.periodType = Timer_PeriodType_MICROSECS;
    timerParams.arg = 1;
    myTimer = Timer_create(2,&axisX_intrrupt_handler,&timerParams,&eb1);
    if (myTimer == NULL) {
        System_abort("Timer create failed");
    }
    Timer_start(myTimer);
#endif
    //---------------
    PinoutSet();

    //
    // Enable and wait for the port to be ready for access
    //
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
//    {
//    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2))
    {
    }

#ifdef TIVA
//    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR + TIMER_CFG_B_PWM );
//    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR);


//    TimerConfigure(TIMER0_BASE, TIMER_CFG_B_PWM);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR + TIMER_CFG_A_PWM);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR + TIMER_CFG_B_PWM);

//    TimerMatchSet(TIMER0_BASE, TIMER_B, 0x002F);
    TimerMatchSet(TIMER1_BASE, TIMER_BOTH, 0x002F);

//    TimerLoadSet(TIMER0_BASE, TIMER_B, 0xFFFF);
    TimerLoadSet(TIMER1_BASE, TIMER_BOTH, 0xFFFF);

// time_ 1 B
    TimerControlLevel(TIMER1_BASE, TIMER_BOTH, true);  // inverse mode
    TimerControlStall(TIMER1_BASE, TIMER_BOTH, true);  // Stop in debug mode.

//    TimerIntRegister(TIMER1_BASE, TIMER_B , &axisX_intrrupt_handler);

//    TimerIntEnable(TIMER1_BASE, TIMER_CAPB_EVENT);

//    TimerPrescaleSet(TIMER0_BASE, TIMER_B, 0xFF);
//    TimerPrescaleMatchSet(TIMER0_BASE, TIMER_B, 0xFF);
//------ end timer_0 B

//    TimerControlLevel(TIMER1_BASE, TIMER_BOTH, true);
//    TimerControlStall(TIMER1_BASE, TIMER_BOTH, true);  // Stop in debug mode.

//    GPIOPinConfigure(GPIO_PF1_T0CCP1);
    GPIOPinConfigure(GPIO_PF2_T1CCP0);
    GPIOPinConfigure(GPIO_PF3_T1CCP1);
    // GPIO_PF3_T1CCP1 green
//    GPIOPinTypeTimer(GPIO_PORTF_BASE,  RED_LED|BLUE_LED|GREEN_LED);

//    TimerEnable(TIMER0_BASE, TIMER_B);    // in testPrepare
    TimerEnable(TIMER1_BASE, TIMER_BOTH);

//    testPrepare();  // Test and debug interrupt

    //
    // Setup the blink functionality
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
    ROM_TimerConfigure(WTIMER5_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR);
//    ROM_TimerConfigure(WTIMER5_BASE, TIMER_CFG_B_PERIODIC);
    ROM_TimerLoadSet64(WTIMER5_BASE, 0xFFFFFFFFFFFFFFFF);
    ROM_IntEnable(INT_WTIMER5B);
//    ROM_TimerIntEnable(WTIMER5_BASE, TIMER_TIMB_TIMEOUT);
    TimerIntRegister(WTIMER5_BASE, TIMER_B , &axisX_intrrupt_handler);
    ROM_TimerEnable(WTIMER5_BASE,TIMER_B);


#endif


#ifdef USE_HW
    //
    // Configure each timer for output mode
    //0x4 For a 16/32-bit timer, this value selects the 16-bit timer configuration.
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CFG) = 0x04;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) = 0x0A;   //
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= 0x0040; //  TABPWML inverted
    HWREG(TIMER_BASE_X_AXIS  + TIMER_O_TAMATCHR) = PORT_PULS_WIDTH;

    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CFG) = 0x04;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBMR) = 0x0A;   //
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= 0x4000; //  TABPWML inverted
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TBMATCHR) = PORT_PULS_WIDTH;

    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CFG) = 0x04;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMR) = 0x0A;   //
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= 0x0040; //  TABPWML inverted
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMATCHR) = PORT_PULS_WIDTH;

    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CFG) = 0x04;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBMR) = 0x0A;   //
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= 0x0040; //  TABPWML inverted
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBMATCHR) = PORT_PULS_WIDTH;

    /**
     * 0x0A = 1010
     *
     *[3] Value Description
     *[3] 0 Capture or compare mode is enabled.
     *[3] 1 PWM mode is enabled.        ^
     * [2] Value Description
     * 0 Edge-Count mode                ^
     * 1 Edge-Time mode
     * [0:1]Value Description
     * 0x0 Reserved
     * 0x1 One-Shot Timer mode
     * 0x2 Periodic Timer mode          ^
     * 0x3 Capture mode
     */


    /**
     * GPTM Timer B Interval Load (GPTMTBILR), offset 0x02C
     */

    //
    // Invert the output signals.
    //
    /**
     * TAPWML - [6]
     * TBPWML - [14]
     * Value Description
     * 0 Output is unaffected.
     * 1 Output is inverted.
     */


    //
    // Enable timers to begin counting
    //
//    ROM_TimerEnable(RED_TIMER_BASE, TIMER_BOTH);
//    ROM_TimerEnable(GREEN_TIMER_BASE, TIMER_BOTH);
//    ROM_TimerEnable(BLUE_TIMER_BASE, TIMER_BOTH);

    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN; //
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN; // green
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN; // blue


//    GPIOPinConfigure(GPIO_PF1_T0CCP1);
//    GPIOPinConfigure(GPIO_PF2_T1CCP0);
//    GPIOPinConfigure(GPIO_PF3_T1CCP1);
    // GPIO_PF3_T1CCP1 green
//    GPIOPinTypeTimer(GPIO_PORTF_BASE,  RED_LED|BLUE_LED|GREEN_LED);


    //
    // Configure the GPIO port for the LED operation.
    //
//    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);


    //
    // Setup the blink functionality
    //
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);

#endif




}



