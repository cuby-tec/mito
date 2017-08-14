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
#define TIMER_BASE_Z_AXIS   TIMER3_BASE
#define TIMER_BASE_E_AXIS   TIMER3_BASE

//PERIPH port
#define TIMER_X_AXIS_PERIPH     SYSCTL_PERIPH_TIMER1
#define TIMER_Y_AXIS_PERIPH     SYSCTL_PERIPH_TIMER1
#define TIMER_Z_AXIS_PERIPH     SYSCTL_PERIPH_TIMER2
#define TIMER_E_AXIS_PERIPH     SYSCTL_PERIPH_TIMER2

#define RED_GPIO_PIN            GPIO_PIN_1
#define BLUE_GPIO_PIN           GPIO_PIN_2
#define GREEN_GPIO_PIN          GPIO_PIN_3


#define X_axis_int      (1) // from axisX_intrrupt_handler to orderlyTask
#define X_axis_int_fin  (2) // from axisX_intrrupt_handler to orderlyTask


#define PORT_PULS_WIDTH     0x04FF


#define N_AXIS  4 // Number of axes

#define X_AXIS  0 // Axis indexing value
#define Y_AXIS  1
#define Z_AXIS  2
#define E_AXIS  3

#define X_FLAG  (1<<X_AXIS)
#define Y_FLAG  (1<<Y_AXIS)
#define Z_FLAG  (1<<Z_AXIS)
#define E_FLAG  (1<<E_AXIS)

#endif /* MSMOTOR_MSPORT_H_ */
