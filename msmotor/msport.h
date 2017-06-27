/*
 * msport.h
 *
 *  Created on: 27 июн. 2017 г.
 *      Author: walery
 *
 *     Назначение портов шаговых двигателей.
 */

#ifndef MSMOTOR_MSPORT_H_
#define MSMOTOR_MSPORT_H_

//BASE port
#define TIMER_BASE_X_AXIS   TIMER1_BASE
#define TIMER_BASE_Y_AXIS   TIMER1_BASE
#define TIMER_BASE_Z_AXIS   TIMER2_BASE
#define TIMER_BASE_E_AXIS   TIMER2_BASE

//PERIPH port
#define TIMER_X_AXIS_PERIPH     SYSCTL_PERIPH_TIMER1
#define TIMER_Y_AXIS_PERIPH     SYSCTL_PERIPH_TIMER1
#define TIMER_Z_AXIS_PERIPH     SYSCTL_PERIPH_TIMER2
#define TIMER_E_AXIS_PERIPH     SYSCTL_PERIPH_TIMER2


#define PORT_PULS_WIDTH     0x2F

#endif /* MSMOTOR_MSPORT_H_ */
