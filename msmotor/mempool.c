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

uint32_t sync[2];


struct Stepper_state_t sts;
struct Stepper_state_t sts_y;
struct Stepper_state_t sts_z;
struct Stepper_state_t sts_e;

struct sSegment sector[SECTOR_SIZE];

//static struct Stepper_state_t* psts = &sts;

//struct sControl block;
//struct sControl block_y;
//struct sControl block_z;
//struct sControl block_e;

uint32 current_segment;

struct sControl* pblock;
struct sControl* pblock_y;
struct sControl* pblock_z;
struct sControl* pblock_e;

struct sSegment* segment = &sector[0];


uint32_t speedRate[N_AXIS];

const struct sControl default_block = {
   X_AXIS, 10, 2, 3, 7, 50132, 0, 5370, 3, 50132, 0, 1, 2, 3, forward
};

const struct sHead default_segment_head = {
   1,N_AXIS, (X_FLAG|Y_FLAG|Z_FLAG|E_FLAG)
};

uint16_t axis_flags; //X_FLAG, Y_FLAG, Z_FLAG, E_FLAG

//block_state* current_block = &block;

//stepper_state sts;

//-------------- Function


void createCommandPool(){
#ifdef KTF7
	OS_MEMF_Create(&cmdPool,block_buffer,BLOCK_BUFFER_SIZE,sizeof(block_state));
#else
	NoOperation;
#endif

}


