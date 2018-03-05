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
#include "drivers/SPI_Microstepper.h"
#include "Microstep.h"

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
#include "memf/cnc_sector.h"

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

//-------------------- initBlock & sSector. for test purposes.
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

}

union uMicrostep usteps;


void disableMotors(void)
{
    usteps.microsteps.EN = 0x3F;
    SPI_Send(usteps.data, MSG_LENGTH);
}

/**
 * Загрузка микрошага в порт микрошага драйверов.
 */
void uploadMicrosteps(struct sSegment* segment){
//    uint8_t msdata[N_AXIS];
    uint8_t j;

    for(j=0;j<4;j++)
        usteps.data[j] = 0;

    usteps.microsteps.X = segment->axis[X_AXIS].microsteps;
    usteps.microsteps.Y = segment->axis[Y_AXIS].microsteps;
    usteps.microsteps.Z = segment->axis[Z_AXIS].microsteps;
    usteps.microsteps.E0 = segment->axis[E_AXIS].microsteps;
//    usteps.microsteps.EN |= EN_E1;  \\ Disabled
//    SPI_Send(msdata, N_AXIS);
    SPI_Send(usteps.data, MSG_LENGTH);
}

void updateDirections(struct sSegment* segment)
{
    if(segment->axis[X_AXIS].direction == forward)
        GPIOPinWrite(DIRECTION_PORT, DIR_X, DIR_X);
    else
        GPIOPinWrite(DIRECTION_PORT, DIR_X, 0);

    if(segment->axis[Y_AXIS].direction == forward)
        GPIOPinWrite(DIRECTION_PORT, DIR_Y, DIR_Y);
    else
        GPIOPinWrite(DIRECTION_PORT, DIR_Y, 0);

    if(segment->axis[Z_AXIS].direction == forward)
        GPIOPinWrite(DIRECTION_PORT, DIR_Z, DIR_Z);
    else
        GPIOPinWrite(DIRECTION_PORT, DIR_Z, 0);

    if(segment->axis[E_AXIS].direction == forward)
        GPIOPinWrite(DIRECTION_PORT, DIR_E, DIR_E);
    else
        GPIOPinWrite(DIRECTION_PORT, DIR_E, 0);

}

/**
 * инициализация нового сегмента.
 *
 */
void pblockSegment(struct sSegment* segment)
{
    pblock = &segment->axis[X_AXIS];
    pblock_y = &segment->axis[Y_AXIS];
    pblock_z = &segment->axis[Z_AXIS];
    pblock_e = &segment->axis[E_AXIS];
    uploadMicrosteps(segment);
    updateDirections(segment);
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
    mask_axis[0] = 0;
    mask_axis[1] = 0;

//======== SPI initializing
    //
    // Enable the SSI0 peripheral
    //
    SysCtlPeripheralEnable(EXT_PRIPHERAL);
    //
    // Wait for the SSI0 module to be ready.
    //
    while(!SysCtlPeripheralReady(EXT_PRIPHERAL))
    {
    }
    SPI_init();
//-------DEbug
#define UMICROSTEP_no  //MEANDR //ONE

#ifdef MEANDR
    uint8_t tx_array[MSG_LENGTH] = {0xAA,0xAA,0xAA};
    uint8_t tx_array1[MSG_LENGTH] = {0x55,0x55,0x55};
    while(1){
        SPI_Send(tx_array, MSG_LENGTH);
        SPI_Send(tx_array1, MSG_LENGTH);
    }
#endif
#ifdef ONE
    uint8_t tx_array[MSG_LENGTH] = {0x80,0x00,0x00};// e1:EN
    uint8_t tx_array1[MSG_LENGTH] = {0x00,0x00,0x01};//x:MS1
    while(1){
        SPI_Send(tx_array, MSG_LENGTH);
        SPI_Send(tx_array1, MSG_LENGTH);
    }

#endif
#ifdef UMICROSTEP
    uint8_t j;
    uint32_t rx;
    union uMicrostep usteps;

    for(j=0;j<4;j++)
        usteps.data[j] = 0;
//    union uMicrostep usteps;

    while(1){
        usteps.microsteps.X |= (MS1|MS2);
        usteps.microsteps.EN |= (EN_X);
        SPI_Send(usteps.data, MSG_LENGTH);
        usteps.microsteps.X &= ~(MS1|MS2);
        usteps.microsteps.EN &= ~(EN_X);
        SPI_Send(usteps.data, MSG_LENGTH);
        rx=0;
        SPI_Read(&rx, 1);
    }
#else
    uint8_t j;
    uint32_t rx;
    union uMicrostep usteps;

    for(j=0;j<4;j++)
        usteps.data[j] = 0;

    usteps.microsteps.EN |= (EN_X+EN_Y+EN_Z+EN_E0+EN_E1);

    SPI_Send(usteps.data, MSG_LENGTH);
    rx=0;
    SPI_Read(&rx, 1);
#endif
//================ end initialization

}



