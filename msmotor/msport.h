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

#define SEGMENT_QUEE_SIZE     10 //2 10

// Время ожидания получения места для нового Сегмента
// По истечении времени поисходит отправка состояния без загрузки сегмента в Очередь Сегментов.
#define SEGMENT_DELAY       10

#define TIMER_X     TIMER_A
#define TIMER_Y     TIMER_B
#define TIMER_Z     TIMER_A
#define TIMER_E     TIMER_B

//TIMER BASE port
#define TIMER_BASE_X_AXIS   TIMER1_BASE
#define TIMER_BASE_Y_AXIS   TIMER1_BASE
#define TIMER_BASE_Z_AXIS   TIMER2_BASE
#define TIMER_BASE_E_AXIS   TIMER2_BASE


//TIMER EN
#define TIMER_X_AXIS_EN     TIMER_CTL_TAEN
#define TIMER_Y_AXIS_EN     TIMER_CTL_TBEN
#define TIMER_Z_AXIS_EN     TIMER_CTL_TAEN
#define TIMER_E_AXIS_EN     TIMER_CTL_TBEN


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
#define TIMER_HOTEND_PERIPH     SYSCTL_PERIPH_WTIMER0

#define RED_GPIO_PIN            GPIO_PIN_1
#define BLUE_GPIO_PIN           GPIO_PIN_2
#define GREEN_GPIO_PIN          GPIO_PIN_3



#define DIRECTION_PORT      GPIO_PORTA_BASE
#define DIR_X         GPIO_PIN_5
#define DIR_Y         GPIO_PIN_4
#define DIR_Z         GPIO_PIN_3
#define DIR_E         GPIO_PIN_2


#define PORT_PULS_WIDTH     0x04FF

#define SVI_PRIORITY    3 // low PORT_A_P0

#define INT_TIMER1_X_PRIORITY   2
#define INT_TIMER_Y_PRIORITY    2
#define INT_TIMER1_Z_PRIORITY   2
#define INT_TIMER_E_PRIORITY    2
#define INT_HOTEND_TIMER_PRIORITY   3


#define N_AXIS  4 // Number of axes nuts_bolts

#define X_AXIS  0 // Axis indexing value
#define Y_AXIS  1
#define Z_AXIS  2
#define E_AXIS  3

#define X_FLAG  (1<<X_AXIS)
#define Y_FLAG  (1<<Y_AXIS)
#define Z_FLAG  (1<<Z_AXIS)
#define E_FLAG  (1<<E_AXIS)

#define EXT_PORT        SSI2_BASE
#define EXT_PRIPHERAL   SYSCTL_PERIPH_SSI2
#define EXT_SHLD        GPIO_PIN_6  //GPIO_PORTA_BASE, GPIO_PIN_6
#define EXT_RCLK        GPIO_PIN_7  //GPIO_PORTA_BASE, GPIO_PIN_7
#define EXT_BASE        GPIO_PORTA_BASE

// Enders
#define ENDER_BASE      GPIO_PORTE_BASE

#define ENDER_X_MIN     GPIO_PIN_4
#define ENDER_X_MAX     GPIO_PIN_5

#define ENDER_Y_MIN     GPIO_PIN_2
#define ENDER_Y_MAX     GPIO_PIN_3

#define ENDER_Z_MIN     GPIO_PIN_0
#define ENDER_Z_MAX     GPIO_PIN_1

#define ENDER_ACTIVE    true
#define ENDER_PASSIVE   false


// Hotend =========

#define HOTEND_PORT_BASE     GPIO_PORTC_BASE
#define HOTEND_SWITCH   GPIO_PIN_5

#define HOTEND_THERMO_BASE      ADC0_BASE

#define HOTEND_THERMO_PORT_BASE GPIO_PORTD_BASE
#define HOTEND_THERMO_INPUT     GPIO_PIN_2

#define TIMER_BASE_HOTEND   WTIMER0_BASE

#define TIMER_HOTEND_EN     TIMER_CTL_TBEN

#define INT_HOTEND_TIMER    INT_WTIMER0B

#define ADC_HOTEND_PERIPH   SYSCTL_PERIPH_ADC0
#define ADC_HOTEND_BASE     ADC0_BASE
#define SS3                 3


#endif /* MSMOTOR_MSPORT_H_ */
