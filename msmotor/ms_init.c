/*
 * ms_init.c
 *
 *  Created on: 19 июн. 2017 г.
 *      Author: walery
 */

//#include <stdint.h>
//#include <stdbool.h>
//
//#include "inc/hw_gpio.h"
//#include "inc/hw_types.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/pin_map.h"
//#include "driverlib/timer.h"
//#include "driverlib/gpio.h"
//#include "driverlib/interrupt.h"
//
//#include "inc/hw_types.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_timer.h"
//#include "inc/hw_ints.h"
//#include "driverlib/sysctl.h"
//#ifdef TARGET_IS_TM4C123_RA1
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
//#endif
//

#include "drivers/pinout.h"

#include "ms_init.h"
#include "ms_model.h"
#include "msport.h"
//#include "packages/ti/sysbios/hal/Hwi.h"
//#include "xdc/runtime/Error.h"
//#include "xdc/runtime/System.h"
//#include "ti/sysbios/hal/Timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "orderlyTask.h"


//------------- defs

//#define TIVA
#define USE_HW


struct Ms_delay ms_delay;

//-------- vars
uint32_t static pid;
//-------------- function

void rgb_enable(void){
    //
    // Configure the GPIO Pin Mux for PF1
    // for T0CCP1
    //
//    MAP_GPIOPinConfigure(GPIO_PF1_T0CCP1);
//    MAP_GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_1);//red

    //
    // Configure the GPIO Pin Mux for PF2
    // for T1CCP0
    //
/*
    MAP_GPIOPinConfigure(GPIO_PF2_T1CCP0);
    MAP_GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_2);//blue
*/

    //
    // Configure the GPIO Pin Mux for PF3
    // for T1CCP1
    //
/*
    MAP_GPIOPinConfigure(GPIO_PF3_T1CCP1);
    MAP_GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_3);//green
*/
    pid = 3;
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_GPIO_PIN|GREEN_GPIO_PIN|BLUE_GPIO_PIN);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);//синхро-импульс состяния Задачи
//    HWREG(GPIO_PORTF_BASE + GPIO_O_DATA) = BLUE_GPIO_PIN;
    return;
}

void rgb_disable(void){
    //
    // Configure the GPIO pads as general purpose inputs.
    //
    pid = 38;
//    ROM_GPIOPinTypeGPIOInput(RED_GPIO_BASE, RED_GPIO_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, RED_GPIO_PIN|GREEN_GPIO_PIN|BLUE_GPIO_PIN);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
//    HWREG(GPIO_PORTF_BASE + GPIO_O_DATA) &= ~BLUE_GPIO_PIN;
    return;
}




void Timer_Y_isr(void){
    static uint32_t cnt_y;
    cnt_y++;
    TimerIntClear(TIMER_BASE_Y_AXIS, TIMER_CAPB_EVENT);
    if(cnt_y & 1){
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_GPIO_PIN, GREEN_GPIO_PIN);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE, GREEN_GPIO_PIN, ~GREEN_GPIO_PIN);
    }
}

// Z axis
void Timer_Z_isr(void){
    static uint32_t cnt_z;
    TimerIntClear(TIMER_BASE_Z_AXIS, TIMER_CAPA_EVENT);
    cnt_z++;
    if(cnt_z & 1){
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_GPIO_PIN, BLUE_GPIO_PIN);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_GPIO_PIN, 0);
    }
}

// E axis
void Timer_E_isr(void){
    static uint32_t cnt_e;
    TimerIntClear(TIMER_BASE_E_AXIS, TIMER_CAPB_EVENT);
    cnt_e++;
    if(pid != 38){
        if(cnt_e &1){
            //        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,BLUE_GPIO_PIN);

            HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) &= ~BLUE_GPIO_PIN ;
        }else{
            //        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_GPIO_PIN|GREEN_GPIO_PIN|BLUE_GPIO_PIN);
            HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= BLUE_GPIO_PIN;
        }
    }
}



// ISR WTIMER5======================
void Timer_callback(void){
    static uint32_t cnt_int = 0;
    TimerIntClear(WTIMER5_BASE,TIMER_TIMB_MATCH+TIMER_TIMB_TIMEOUT+TIMER_CAPB_EVENT);//TIMER_TIMB_TIMEOUT
    //------ task delay

    ms_delay.cnt++;
    ms_delay.counter = TimerValueGet(WTIMER5_BASE, TIMER_B);

    cnt_int++;
    if(( cnt_int & 1)){
//        rgb_enable();
        xTaskNotifyFromISR(orderlyHandling,0x01,eSetBits,NULL);

    }else{
//        rgb_disable();
        xTaskNotifyFromISR(orderlyHandling,0x02,eSetBits,NULL);
    }
}

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
    // Configure each timer for output mode T1A
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN; //
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) = TIMER_TAMR_TAMR_PERIOD|TIMER_TAMR_TAAMS|TIMER_TAMR_TAPWMIE;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = 0x0FFF;    //
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAPWML|TIMER_CTL_TAEVENT_NEG; //  TABPWML inverted
    HWREG(TIMER_BASE_X_AXIS  + TIMER_O_TAMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_IMR) |= TIMER_IMR_CAEIM;
    IntEnable(INT_TIMER1A); // NVIC register setup.
    TimerIntClear(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);

    // timer axis Y = T1B
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //

    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBMR) = TIMER_TBMR_TBMR_PERIOD|TIMER_TBMR_TBAMS|TIMER_TBMR_TBPWMIE;//0x0A;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TBILR) = 0x0FFE;    //
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBPWML|TIMER_CTL_TBEVENT_NEG;//0x4000; TABPWML inverted
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TBMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_IMR) |= TIMER_IMR_CBEIM;
    IntEnable(INT_TIMER1B); // NVIC register setup.
    TimerIntClear(TIMER_BASE_Y_AXIS, TIMER_CAPB_EVENT);

    // timer axis Z = T2A
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN; //
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMR) = TIMER_TAMR_TAMR_PERIOD|TIMER_TAMR_TAAMS|TIMER_TAMR_TAPWMIE;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAILR) = 0x0FFD;    //
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAPWML|TIMER_CTL_TAEVENT_NEG; // TABPWML inverted
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_IMR) |= TIMER_IMR_CAEIM;
    IntEnable(INT_TIMER2A); // NVIC register setup.
    TimerIntClear(TIMER_BASE_Z_AXIS, TIMER_CAPA_EVENT);

    // timer axis E = T2B
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBMR) = TIMER_TBMR_TBMR_PERIOD|TIMER_TBMR_TBAMS|TIMER_TBMR_TBPWMIE;//0x0A;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBILR) = 0x1FFC;    //
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBPWML|TIMER_CTL_TBEVENT_NEG; //  TABPWML inverted
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_IMR) |= TIMER_IMR_CBEIM;
//    IntEnable(INT_TIMER2B); // NVIC register setup.
    TimerIntClear(TIMER_BASE_E_AXIS, TIMER_CAPB_EVENT);

    //
    // Enable timers to begin counting
    //
//    ROM_TimerEnable(RED_TIMER_BASE, TIMER_BOTH);
//    ROM_TimerEnable(GREEN_TIMER_BASE, TIMER_BOTH);
//    ROM_TimerEnable(BLUE_TIMER_BASE, TIMER_BOTH);

//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN; //
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN; // green
//    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN; // blue
//    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN; //


    //
    // Setup the blink functionality
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
/*    TimerEnable(WTIMER5_BASE, ~TIMER_B);
    TimerConfigure(WTIMER5_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR);
//    HWREG(WTIMER5_BASE + TIMER_O_TBMR) = 0x0012;
    HWREG(WTIMER5_BASE + TIMER_O_TBMR) = 0x0002;
    TimerLoadSet64(WTIMER5_BASE, 0x002FFFFFFFFFFFFF);
    TimerIntClear(WTIMER5_BASE,TIMER_TIMB_TIMEOUT);//TIMER_TIMB_TIMEOUT
    IntEnable(INT_WTIMER5B);
    TimerIntEnable(WTIMER5_BASE, TIMER_TIMB_TIMEOUT);
    TimerEnable(WTIMER5_BASE,TIMER_B);
*/

    HWREG(WTIMER5_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //
    HWREG(WTIMER5_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT; // 32 bit
    HWREG(WTIMER5_BASE + TIMER_O_TBMR) = TIMER_TBMR_TBMR_PERIOD|TIMER_TBMR_TBAMS|TIMER_TBMR_TBPWMIE;//0x0A;
    HWREG(WTIMER5_BASE + TIMER_O_TBILR) = 0x003FFFFF;    //
    HWREG(WTIMER5_BASE + TIMER_O_CTL) |= TIMER_CTL_TBPWML|TIMER_CTL_TBEVENT_NEG; //  TABPWML inverted
    HWREG(WTIMER5_BASE + TIMER_O_TBMATCHR) = PORT_PULS_WIDTH*100;
    HWREG(WTIMER5_BASE + TIMER_O_IMR) |= TIMER_IMR_CBEIM;
    TimerIntClear(WTIMER5_BASE, TIMER_CAPB_EVENT);
    IntEnable(INT_WTIMER5B); // NVIC register setup.
    TimerEnable(WTIMER5_BASE,TIMER_B);
    NoOperation;

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



