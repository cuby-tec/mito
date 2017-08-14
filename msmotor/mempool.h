/*
 * mempool.h
 *
 *  Created on: 26.09.2010
 *      Author: walery
 */

#ifndef MEMPOOL_H_
#define MEMPOOL_H_


#include <grbl/Block_command.h>
#ifdef KTF7
#include <rtos.h>
#endif
#include <msmotor/block_state.h>

#include "mscontrol.h"
#include "stepper_state.h"
///------------- Defs


// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
  #define BLOCK_BUFFER_SIZE 18
#endif


//----------- Vars

extern struct sControl* pblock;

//extern struct Stepper_state_t sts;

extern uint16_t axis_flags;

extern block_state* current_block;

extern stepper_state sts;



#ifdef KTF7
extern OS_MEMF cmdPool;
#else
// stub
uint16_t cmdPool[10];
#endif
// A ring buffer for motion instructions
//extern block_t block_buffer[BLOCK_BUFFER_SIZE];
//
//extern char ff_filebuffer[14][9];
extern union{

  char ff_filebuffer[14][12];
  block_state block_buffer[BLOCK_BUFFER_SIZE];

};




//-------- Functions


void createCommandPool();



#endif /* MEMPOOL_H_ */
