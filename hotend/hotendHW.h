/*
 * hotendHW.h
 *
 *  Created on: 23 мар. 2018 г.
 *      Author: walery
 */

#ifndef HOTEND_HOTENDHW_H_
#define HOTEND_HOTENDHW_H_

#include "inc/typedefs.h"

//------------- defs

#define HOTEND_D10_PERIOD   0xC355

//-------------- vars


//--------------- function

extern void initHotendHW(void);

extern uint32_t get_hotend_adc(void);


// interrupt handler WT0CCP1
extern void intHotendHandler(void);

extern void setTIMER_HOTEND(uint32_t value);


#endif /* HOTEND_HOTENDHW_H_ */
