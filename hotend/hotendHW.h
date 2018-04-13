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

//-------------- vars


//--------------- function

extern void initHotendHW(void);

extern uint32_t get_hotend_adc(void);


// interrupt handler WT0CCP1
extern void intHotendHandler(void);




#endif /* HOTEND_HOTENDHW_H_ */
