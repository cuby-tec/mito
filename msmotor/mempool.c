/*
 * mempool.c
 *
 *  Created on: 26.09.2010
 *      Author: Администратор
 */

#include "mempool.h"



//-------- Vars
#ifdef KTF7
OS_MEMF cmdPool;
#else
// stub
uint16_t cmdPool[10];
#endif
union{

	char ff_filebuffer[14][12];
	block_state block_buffer[BLOCK_BUFFER_SIZE];

};


struct Stepper_state_t sts;
//static struct Stepper_state_t* psts = &sts;

static  struct sControl block;
struct sControl* pblock = &block;

uint16_t axis_flags; //X_FLAG, Y_FLAG, Z_FLAG, E_FLAG

block_state* current_block = &block;

stepper_state sts;

//-------------- Function


void createCommandPool(){
#ifdef KTF7
	OS_MEMF_Create(&cmdPool,block_buffer,BLOCK_BUFFER_SIZE,sizeof(block_state));
#else
	NoOperation;
#endif

}


