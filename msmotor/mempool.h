/*
 * mempool.h
 *
 *  Created on: 26.09.2010
 *      Author: walery
 */

#ifndef MEMPOOL_H_
#define MEMPOOL_H_



//#include <grbl/Block_command.h>
#ifdef KTF7
#include <rtos.h>
#endif
//#include <msmotor/block_state.h>
//#include <exchange/ComData_t.h>/
#include "exchange/ComDataReq_t.h"
#include "sSegment.h"
#include "stepper_state.h"
#include "msport.h"
#include "ms_state.h"
///------------- Defs


// The number of linear motions that can be in the plan at any give time
#ifndef SEGMENT_QUEE_SIZE
  #define BLOCK_BUFFER_SIZE 18
#endif


//----------- Vars

extern struct sControl* pblock;

//extern struct Stepper_state_t sts;

extern uint16_t axis_flags;

extern uint32_t g_ui32Flags;

//extern block_state* current_block;

extern uint32_t sync[2];

//extern stepper_state sts;
extern struct Stepper_state_t sts;
extern struct Stepper_state_t sts_y;
extern struct Stepper_state_t sts_z;
extern struct Stepper_state_t sts_e;

extern uint32 current_segment;

extern struct sSegment sector[SEGMENT_QUEE_SIZE];
extern struct sSegment* segment;    // укзатель на текущий сегмент в sSector
//extern struct sControl block;

extern struct sControl* pblock;
extern struct sControl* current_block;

extern struct sControl* pblock_y;
extern struct sControl* pblock_z;
extern struct sControl* pblock_e;

extern uint32_t speedRate[N_AXIS];

extern const struct sControl default_block;

extern const struct sHead default_segment_head;


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
//extern union{
//
//  char ff_filebuffer[14][12];
//  block_state block_buffer[BLOCK_BUFFER_SIZE];
//
//};


// буфер накопления команды Хоста.
extern uint8_t cmdBuffer_usb[sizeof(struct ComDataReq_t)];

// буфер передачи сегмента от Хоста
extern uint8_t segmentBuffer[sizeof(struct sSegment)];

extern struct sMs_State* pMs_State;

extern uint8_t rcvd_SegmentFlag;

//-------- Functions


void createCommandPool();



#endif /* MEMPOOL_H_ */
