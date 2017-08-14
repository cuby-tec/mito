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
#include "mempool.h"
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
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);//синхро-импульс состяния Задачи PE0
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
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);   //         PE0
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
        GPIOPinWrite(GPIO_PORTF_BASE, BLUE_GPIO_PIN, ~BLUE_GPIO_PIN);
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
//            HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) &= ~BLUE_GPIO_PIN ;
            GPIOPinWrite(GPIO_PORTF_BASE, RED_GPIO_PIN, RED_GPIO_PIN);
        }else{
            //        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_GPIO_PIN|GREEN_GPIO_PIN|BLUE_GPIO_PIN);
//            HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= BLUE_GPIO_PIN;
            GPIOPinWrite(GPIO_PORTF_BASE, RED_GPIO_PIN, ~RED_GPIO_PIN);
        }
    }
}



// ISR WTIMER5======================
void Timer_callback(void)
{
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

//-------------------- initBlock
void initBlock(void)
{
    pblock->axis = 0;
    pblock->linenumber = 1;
    pblock->steps   = 10;
    pblock->microsteps = 2;
    pblock->accelerate_until = 3;
#ifdef DOUBLE
    pblock->decelerate_after = 38;
#else
    pblock->decelerate_after = 7;
#endif
#ifdef DOUBLE
    pblock->initial_rate = 50132.6;
#else
    pblock->initial_rate = 50132;
#endif
    pblock->initial_speedLevel = 0;
#ifdef DOUBLE
    pblock->nominal_rate = 5370.2;
#else
    pblock->nominal_rate = 5370;
#endif
    pblock->speedLevel = 3;
#ifdef DOUBLE
    pblock->final_rate = 50132.6;//5676;
#else
    pblock->final_rate = 50132;//5676;
#endif
    pblock->final_speedLevel = 0;
    pblock->schem[0] = 1;
    pblock->schem[1] = 2;
    pblock->schem[2] = 3;
    pblock->direction = forward;
}

//--------------------- initStepper
void initStepper(void)
{
    sts.counter_y = 0;
    sts.point_y = pblock->steps;
    sts.rate_y = pblock->initial_rate;
    sts.state = 0;      //  pblock->schem[0];
    sts.speedLevel = pblock->initial_speedLevel;
}


//*****************************************************************************
//
// Initializes the Timer and GPIO functionality
//
//
//*****************************************************************************
void msInit(void){

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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3))
    {
    }



#ifdef USE_HW
    //
    // Configure each timer for output mode T1A
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN; //
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) = TIMER_TAMR_TAMR_PERIOD|TIMER_TAMR_TAAMS|TIMER_TAMR_TAPWMIE;
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAMIE;
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAPLO;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAILD;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAMRSU;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_IMR) =  TIMER_IMR_CAEIM;// TIMER_IMR_TAMIM; // | TIMER_IMR_TATOIM;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_X_AXIS  + TIMER_O_TAPR) = 0;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TASTALL;
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEVENT_NEG;     //TIMER_CTL_TAEVENT_POS ;
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEVENT_POS;
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAPWML; //  TABPWML inverted
    HWREG(TIMER_BASE_X_AXIS  + TIMER_O_TAMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_X_AXIS  + TIMER_O_TAPMR) = 0;

    TimerIntClear(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);
    IntEnable(INT_TIMER1A); // NVIC register setup.


    // timer axis Y = T1B
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //

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
//    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN; // green
//    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN; // blue
//    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN; //


    //
    // Setup the blink functionality
    //
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
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

/*
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
*/

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



