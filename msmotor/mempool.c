/*
 * mempool.c
 *
 *  Created on: 26.09.2010
 *      Author: Администратор
 */

#include "mempool.h"
#include "msmotor/block_state.h"
#include "exchange/ComDataReq_t.h"

///---------- derfs

//segment_QUEUE_SIZE, segment_ITEM_SIZE
#define segment_QUEUE_SIZE  10
#define segment_ITEM_SIZE   sizeof(struct sSegment)

//-------- Vars
#ifdef KTF7
OS_MEMF cmdPool;
#else
// stub
uint16_t cmdPool[10];


#endif
/*
union{
	char ff_filebuffer[14][12];
	block_state block_buffer[SEGMENT_QUEE_SIZE];
};
*/

uint32_t mask_axis[2];


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


#define DEFAULT_STEPS   10000
#define DEFAULT_ACCELERATEUNTIL 3
#define DEFAULT_initial_rate    141798//70898//50132
#define DEFAULT_nominal_rate    37994//18997//13382//5370
#define DEFAULT_final_rate      141798//70898//50132

uint32_t speedRate[N_AXIS];

struct sControl default_block = {
   X_AXIS, 2, 0, 3,
   DEFAULT_STEPS,
   DEFAULT_ACCELERATEUNTIL,
   DEFAULT_STEPS - DEFAULT_ACCELERATEUNTIL ,
   DEFAULT_initial_rate,
   DEFAULT_nominal_rate,
   DEFAULT_final_rate,
   0, 1, 2, 3, forward, 0, 0
};

const struct sHead default_segment_head = {
   N_AXIS, (X_FLAG|Y_FLAG|Z_FLAG|E_FLAG), 0xFFFF, 1
};

uint16_t axis_flags; //X_FLAG, Y_FLAG, Z_FLAG, E_FLAG

//block_state* current_block = &block;

//stepper_state sts;


uint8_t cmdBuffer_usb[sizeof(struct ComDataReq_t)];

//uint8_t segmentBuffer[sizeof(struct sSegment)];

static struct sMs_State ms_state;

struct sMs_State* pMs_State = &ms_state;

uint8_t rcvd_SegmentFlag = 0;


QueueHandle_t segmentQueue;

//-------------- Function


bool
createCommandPool(){
#ifdef KTF7
	OS_MEMF_Create(&cmdPool,block_buffer,BLOCK_BUFFER_SIZE,sizeof(block_state));
#else

	segmentQueue = xQueueCreate( segment_QUEUE_SIZE, segment_ITEM_SIZE );

	if(segmentQueue == NULL)
	{
	    return (false);
	}else
	{
	    return (true);
	}

	NoOperation;
#endif

}


