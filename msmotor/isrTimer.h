/*
 * isrTimer_X.h
 *
 *  Created on: 1 июл. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_ISRTIMER_H_
#define MSMOTOR_ISRTIMER_H_

//----------------function


extern void Timer_X_isr(void);

extern void Timer1IntHandler(void);
extern void TimerEIntHandler(void);
extern void TimerYIntHandler(void);
extern void TimerZIntHandler(void);

#endif /* MSMOTOR_ISRTIMER_H_ */
