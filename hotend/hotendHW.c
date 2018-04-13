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
#include "driverlib/adc.h"
#include "msmotor/msport.h"
//#include "inc/typedefs.h"

//------------- defs
/**
 * Oversampling is accomplished by averaging multiple
 * samples from the same analog input. Six different
 * oversampling rates are supported; 2x, 4x,
 * 8x, 16x, 32x, and 64x. Specifying an oversampling
 * factor of zero disables hardware oversam-
 * pling.
 */
#define OVERSAMPLING    64

//-------------- vars

    //
    // This array is used for storing the data read from the ADC FIFO. It
    // must be as large as the FIFO for the sequencer in use.  This example
    // uses sequence 3 which has a FIFO depth of 1.  If another sequence
    // was used with a deeper FIFO, then the array size must be changed.
    //
static    uint32_t pui32ADC0Value[1];


//-------------- function

uint32_t get_hotend_adc(void)
{
    //
    // Trigger the ADC conversion.
    //
    ADCProcessorTrigger(ADC_HOTEND_BASE, SS3);

    //
    // Wait for conversion to be completed.
    //
    while(!ADCIntStatus(ADC_HOTEND_BASE, SS3, false))
    {
    }

    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC_HOTEND_BASE, 3);

    //
    // Read ADC Value.
    //
    ADCSequenceDataGet(ADC_HOTEND_BASE, SS3, pui32ADC0Value);

    return pui32ADC0Value[0];
}

static
void _init_ADC(void)
{
    //
    // The ADC0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(ADC_HOTEND_PERIPH);

    // Configure the GPIO Pin Mux for PD2
    // for AIN5

    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    ADCSequenceConfigure(ADC_HOTEND_BASE, SS3, ADC_TRIGGER_PROCESSOR, 0);


    ADCSequenceStepConfigure(ADC_HOTEND_BASE, SS3, 0, ADC_CTL_CH5 | ADC_CTL_IE |
                             ADC_CTL_END);


    ADCHardwareOversampleConfigure(ADC_HOTEND_BASE, OVERSAMPLING);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, SS3);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC_HOTEND_BASE, SS3);

    //
    // Trigger the ADC conversion.
    //
    ADCProcessorTrigger(ADC_HOTEND_BASE, SS3);

    //
    // Wait for conversion to be completed.
    //
    while(!ADCIntStatus(ADC_HOTEND_BASE, SS3, false))
    {
    }

    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC_HOTEND_BASE, 3);

    //
    // Read ADC Value.
    //
    ADCSequenceDataGet(ADC_HOTEND_BASE, SS3, pui32ADC0Value);

    NoOperation;
}

/**
 * Таймер управления ШИМ ключа нагревателя.
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

    _init_ADC();

    NoOperation;

}




/**
 * Interrupt handler for WT0CCP1-HOTEND
 */
void intHotendHandler(void)
{
    TimerIntClear(TIMER_BASE_HOTEND, TIMER_CAPB_EVENT);
    NoOperation;
}

