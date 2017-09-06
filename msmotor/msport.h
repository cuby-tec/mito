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

#define SECTOR_SIZE     10 //2 10

#define TIMER_X     TIMER_A
#define TIMER_Y     TIMER_B
#define TIMER_Z     TIMER_A
#define TIMER_E     TIMER_B

//BASE port
#define TIMER_BASE_X_AXIS   TIMER1_BASE
#define TIMER_BASE_Y_AXIS   TIMER1_BASE
#define TIMER_BASE_Z_AXIS   TIMER2_BASE
#define TIMER_BASE_E_AXIS   TIMER2_BASE

// interrupt port
#define INT_TIMER_X         INT_TIMER1A
#define INT_TIMER_Y         INT_TIMER1B
#define INT_TIMER_Z         INT_TIMER2A
#define INT_TIMER_E         INT_TIMER2B


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
#define orderly_command (4) // Получена команда по каналу USB.

#define DIRECTION_PORT      GPIO_PORTE_BASE
#define DIRECTION_X         GPIO_PIN_4
#define DIRECTION_Y         GPIO_PIN_3
#define DIRECTION_Z         GPIO_PIN_2
#define DIRECTION_E         GPIO_PIN_1


#define PORT_PULS_WIDTH     0x04FF

#define SVI_PRIORITY    3 // low PORT_A_P0

#define INT_TIMER1_X_PRIORITY   2
#define INT_TIMER_Y_PRIORITY    2
#define INT_TIMER1_Z_PRIORITY   2
#define INT_TIMER_E_PRIORITY    2


#define N_AXIS  4 // Number of axes nuts_bolts

#define X_AXIS  0 // Axis indexing value
#define Y_AXIS  1
#define Z_AXIS  2
#define E_AXIS  3

#define X_FLAG  (1<<X_AXIS)
#define Y_FLAG  (1<<Y_AXIS)
#define Z_FLAG  (1<<Z_AXIS)
#define E_FLAG  (1<<E_AXIS)

#endif /* MSMOTOR_MSPORT_H_ */
