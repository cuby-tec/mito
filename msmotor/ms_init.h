/*
 * ms_init.h
 *
 *  Created on: 19 июн. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_MS_INIT_H_
#define MSMOTOR_MS_INIT_H_


//----------- defs
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3



//--------  function
// extern void RGBInit(uint32_t ui32Enable);
extern void msInit(uint32_t ui32Enable);

extern void rgb_enable(void);

extern void rgb_disable(void);

extern void Timer_callback(void);

extern void Timer_X_isr(void);

extern void Timer_Y_isr(void);

extern void Timer_Z_isr(void);


#endif /* MSMOTOR_MS_INIT_H_ */
