/*
 * mempool.c
 *
 *  Created on: 26.09.2010
 *      Author: Администратор
 */

#include "mempool.h"
#include "msmotor/block_state.h"


//-------- Vars
#ifdef KTF7
OS_MEMF cmdPool;
#else
// stub
uint16_t cmdPool[10];
#endif
union{

	char ff_filebuffer[14][12];
	block_state block_buffer[SEGMENT_QUEE_SIZE];

};

uint32_t sync[2];


struct Stepper_state_t sts;
struct Stepper_state_t sts_y;
struct Stepper_state_t sts_z;
struct Stepper_state_t sts_e;

struct sSegment sector[SEGMENT_QUEE_SIZE];

//static struct Stepper_state_t* psts = &sts;

//struct sControl block;
//struct sControl block_y;
//struct sControl block_z;
//struct sControl block_e;

uint32 current_segment;
struct sSegment* segment = &sector[0];

struct sControl* pblock;
struct sControl* pblock_y;
struct sControl* pblock_z;
struct sControl* pblock_e;



uint32_t speedRate[N_AXIS];

const struct sControl default_block = {
   X_AXIS, 2, 0, 3, 10, 3, 7,  50132, 5370, 50132, 0, 1, 2, 3, forward, 0, 0
};

const struct sHead default_segment_head = {
   N_AXIS, (X_FLAG|Y_FLAG|Z_FLAG|E_FLAG), 0xFFFF, 1
};

uint16_t axis_flags; //X_FLAG, Y_FLAG, Z_FLAG, E_FLAG

//block_state* current_block = &block;

//stepper_state sts;


uint8_t cmdBuffer_usb[sizeof(struct ComDataReq_t)];

uint8_t segmentBuffer[sizeof(struct sSegment)];

static struct sMs_State ms_state;

struct sMs_State* pMs_State = &ms_state;

//-------------- Function


void createCommandPool(){
#ifdef KTF7
	OS_MEMF_Create(&cmdPool,block_buffer,BLOCK_BUFFER_SIZE,sizeof(block_state));
#else
	NoOperation;
#endif

}


