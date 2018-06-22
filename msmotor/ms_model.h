/*
 * ms_model.h
 *
 *  Created on: 21 июн. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_MS_MODEL_H_
#define MSMOTOR_MS_MODEL_H_


#include <stdint.h>
#include "stepper_state.h"
#include "msmotor/msport.h"



//--------- defs
//----  Типы блоков управления
#define MS_BLOCK_END        30

#define MS_COMMAND_SEMA_SET     NoOperation


//--------- vars

extern void (*ms_finBlock)(void);

extern int32_t current_pos[N_AXIS];

//extern stepper_state sts;

//extern uint32_t cnt_int;    // debug counter;

//-------- function

//extern void axisX_intrrupt_handler(void);
//extern Void axisX_intrrupt_handler(UArg arg);


extern void continueBlock(void);

extern void exitBlock(void);

extern void start_t1(uint8_t pusc);
#ifdef commit_13
extern void axisX_rateHandler();
#endif
extern void ms_nextSector();

//extern void start_xkalibrovka();
extern void start_xkalibrovka(uint8_t axle);

//extern void stop_xkalibrovka();
extern void stop_xkalibrovka(uint8_t axle);

//extern void testPrepare(void);

#endif /* MSMOTOR_MS_MODEL_H_ */
