/*
 * hotendHW.h
 *
 *  Created on: 23 мар. 2018 г.
 *      Author: walery
 */

#ifndef HOTEND_HOTENDHW_H_
#define HOTEND_HOTENDHW_H_



//------------- defs

//-------------- vars


//--------------- function

extern void initHotendHW(void);

// interrupt handler WT0CCP1
extern void intHotendHandler(void);

#endif /* HOTEND_HOTENDHW_H_ */
