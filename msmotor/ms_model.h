/*
 * ms_model.h
 *
 *  Created on: 21 июн. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_MS_MODEL_H_
#define MSMOTOR_MS_MODEL_H_


#include <stdint.h>
//#include <std.h>
#include "stepper_state.h"


//--------- defs
//----  Типы блоков управления
#define MS_BLOCK_END        30

#define MS_COMMAND_SEMA_SET     NoOperation

//--------- vars
extern stepper_state sts;

extern uint32_t cnt_int;    // debug counter;

//-------- function

extern void axisX_intrrupt_handler(void);
//extern Void axisX_intrrupt_handler(UArg arg);

extern void testPrepare(void);

#endif /* MSMOTOR_MS_MODEL_H_ */
