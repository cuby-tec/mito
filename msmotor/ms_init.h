/*
 * ms_init.h
 *
 *  Created on: 19 июн. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_MS_INIT_H_
#define MSMOTOR_MS_INIT_H_


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



//----------- defs
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3


//------------- defs
struct Ms_delay{
    uint32_t cnt;
    uint32_t counter;
};


//-------------- vars

extern struct Ms_delay ms_delay;


//--------  function
// extern void RGBInit(uint32_t ui32Enable);
extern void msInit(uint32_t ui32Enable);

extern void rgb_enable(void);

extern void rgb_disable(void);

extern void Timer_callback(void);


extern void Timer_Y_isr(void);

extern void Timer_Z_isr(void);

extern void Timer_E_isr(void);


#endif /* MSMOTOR_MS_INIT_H_ */
