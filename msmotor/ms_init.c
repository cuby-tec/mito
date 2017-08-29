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
#include "cnc_sector.h"

//------------- defs

//#define TIVA
#define USE_HW


struct Ms_delay ms_delay;

//-------- vars
//uint32_t static pid;
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
//    pid = 3;
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_GPIO_PIN|GREEN_GPIO_PIN|BLUE_GPIO_PIN);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);//синхро-импульс состяния Задачи PE0
//    HWREG(GPIO_PORTF_BASE + GPIO_O_DATA) = BLUE_GPIO_PIN;
    return;
}

void rgb_disable(void){
    //
    // Configure the GPIO pads as general purpose inputs.
    //
//    pid = 38;
//    ROM_GPIOPinTypeGPIOInput(RED_GPIO_BASE, RED_GPIO_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, RED_GPIO_PIN|GREEN_GPIO_PIN|BLUE_GPIO_PIN);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);   //         PE0
//    HWREG(GPIO_PORTF_BASE + GPIO_O_DATA) &= ~BLUE_GPIO_PIN;
    return;
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
    struct sSegment* segment;

    init_cncsector();

    current_segment = 0;

    segment = &sector[current_segment];

    pblock = &segment->axis[X_AXIS];
    pblock_y = &segment->axis[Y_AXIS];
    pblock_z = &segment->axis[Z_AXIS];
    pblock_e = &segment->axis[E_AXIS];

/*
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
    pblock->direction = forward;*/
}

//--------------------- initStepper
void initStepper(uint8_t axis)
{
    switch(axis){
    case N_AXIS:
        sts.counter = 0;
        sts.point = pblock->steps;
        sts.rate = pblock->initial_rate;
        sts.state = 0;      //  pblock->schem[0];
        sts.speedLevel = pblock->initial_speedLevel;

        sts_y.counter   = 0;
        sts_y.point   = pblock_y->steps;
        sts_y.rate    = pblock_y->initial_rate;
        sts_y.state     = 0;
        sts_y.speedLevel = pblock_y->initial_speedLevel;

        sts_z.counter   = 0;
        sts_z.point   = pblock_z->steps;
        sts_z.rate    = pblock_z->initial_rate;
        sts_z.state     = 0;
        sts_z.speedLevel = pblock_z->initial_speedLevel;

        sts_e.counter   = 0;
        sts_e.point   = pblock_e->steps;
        sts_e.rate    = pblock_e->initial_rate;
        sts_e.state     = 0;
        sts_e.speedLevel = pblock_e->initial_speedLevel;

        break;
    case X_AXIS:
        sts.counter = 0;
        sts.point = pblock->steps;
        sts.rate = pblock->initial_rate;
        sts.state = 0;      //  pblock->schem[0];
        sts.speedLevel = pblock->initial_speedLevel;
        break;

    case Y_AXIS:
        sts_y.counter   = 0;
        sts_y.point   = pblock_y->steps;
        sts_y.rate    = pblock_y->initial_rate;
        sts_y.state     = 0;
        sts_y.speedLevel = pblock_y->initial_speedLevel;
        break;

    case Z_AXIS:
        sts_z.counter   = 0;
        sts_z.point   = pblock_z->steps;
        sts_z.rate    = pblock_z->initial_rate;
        sts_z.state     = 0;
        sts_z.speedLevel = pblock_z->initial_speedLevel;
        break;

    case E_AXIS:
        sts_e.counter   = 0;
        sts_e.point   = pblock_e->steps;
        sts_e.rate    = pblock_e->initial_rate;
        sts_e.state     = 0;
        sts_e.speedLevel = pblock_e->initial_speedLevel;
        break;
    }
}


//*****************************************************************************
//
// Initializes the Timer and GPIO functionality
//
//
//*****************************************************************************
void msInit(void){
    PinoutSet();

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(TIMER_X_AXIS_PERIPH);
    SysCtlPeripheralEnable(TIMER_Y_AXIS_PERIPH);
    SysCtlPeripheralEnable(TIMER_Z_AXIS_PERIPH);
    SysCtlPeripheralEnable(TIMER_E_AXIS_PERIPH);

//  Timer X initializing. ======================
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
    //
    // Set the Timer1A match value to load value / 3.
    //
//    TimerMatchSet(TIMER_BASE_X_AXIS, TIMER_A,
//                  TimerLoadGet(TIMER_BASE_X_AXIS, TIMER_A) / 3);
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_IMR) |= TIMER_IMR_CAEIM;
    TimerIntClear(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);
    IntEnable(INT_TIMER1A); // NVIC register setup.
    IntPrioritySet(INT_TIMER_X, INT_TIMER1_X_PRIORITY);

//  Timer Y initializing.    ===================
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

// Timer Z initializing. ============================
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN; //
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMR) = TIMER_TAMR_TAMR_PERIOD|TIMER_TAMR_TAAMS|TIMER_TAMR_TAPWMIE;
//    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAMIE;
//    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAPLO;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAILD;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAMR) |= TIMER_TAMR_TAMRSU;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_IMR) =  TIMER_IMR_CAEIM;// TIMER_IMR_TAMIM; // | TIMER_IMR_TATOIM;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_Z_AXIS  + TIMER_O_TAPR) = 0;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TASTALL;
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEVENT_NEG;     //TIMER_CTL_TAEVENT_POS ;
//    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEVENT_POS;
//    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAPWML; //  TABPWML inverted
    HWREG(TIMER_BASE_Z_AXIS  + TIMER_O_TAMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_Z_AXIS  + TIMER_O_TAPMR) = 0;
    //
    // Set the Timer3A match value to load value / 3.
    //
//    TimerMatchSet(TIMER_BASE_Z_AXIS, TIMER_A,
//                  TimerLoadGet(TIMER_BASE_Z_AXIS, TIMER_A) / 3);
//    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_IMR) |= TIMER_IMR_CAEIM;
    TimerIntClear(TIMER_BASE_Z_AXIS, TIMER_CAPA_EVENT);
    IntEnable(INT_TIMER_Z); // NVIC register setup.
    IntPrioritySet(INT_TIMER_Z, INT_TIMER1_Z_PRIORITY);

// Timer E initializing ==========================

    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CFG) = TIMER_CFG_16_BIT;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBMR) = TIMER_TBMR_TBMR_PERIOD|TIMER_TBMR_TBAMS|TIMER_TBMR_TBPWMIE;
//    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TAMR) |= TIMER_TBMR_TAMIE;
//    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TAMR) |= TIMER_TBMR_TAPLO;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBMR) |= TIMER_TBMR_TBILD;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBMR) |= TIMER_TBMR_TBMRSU;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_IMR)  |=  TIMER_IMR_CBEIM;// TIMER_IMR_TAMIM; // | TIMER_IMR_TATOIM;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_TBILR) = 0xFFFF;    //
    HWREG(TIMER_BASE_E_AXIS  + TIMER_O_TAPR) = 0;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBSTALL;
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEVENT_NEG;     //TIMER_CTL_TAEVENT_POS ;
//    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEVENT_POS;
//    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBPWML; //  TABPWML inverted
    HWREG(TIMER_BASE_E_AXIS  + TIMER_O_TBMATCHR) = PORT_PULS_WIDTH;
    HWREG(TIMER_BASE_E_AXIS  + TIMER_O_TBPMR) = 0;

    TimerIntClear(TIMER_BASE_E_AXIS, TIMER_CAPB_EVENT);
    IntEnable(INT_TIMER_E); // NVIC register setup.
    IntPrioritySet(INT_TIMER_E, INT_TIMER_E_PRIORITY);


//=====================
    sync[0] = 0;
    sync[1] = 0;
}



