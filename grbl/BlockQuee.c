/*
 * BlockQuee.c
 * Company: CUBY
 * Project: 052ktf
 *  Created on: 10.12.2012
 *      Author: walery
 */


#include "BlockQuee.h"
#include "Planner_t.h"
#include <Ktf_model.h>
//--- Defs


//----------------- Vars 

volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
volatile uint8_t block_buffer_tail;       // Index of the block to process now
static uint8_t next_buffer_head;                 // Index of the next buffer head

//block_t* cmdQuee[BLOCK_BUFFER_SIZE];
block_state* cmdQuee[BLOCK_BUFFER_SIZE];

//------------------ Functions





// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
uint8_t next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}

// Returns the index of the previous block in the ring buffer
uint8_t prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}

/*
static void plan_reset_buffer()
{
	block_buffer_head = 0;
	next_buffer_head = next_block_index(block_buffer_head);
	block_buffer_tail = next_buffer_head;
}
*/


/**
 * Return the pointer of the next block.
 * ? if(cmdPool.pFree == (void*)1)
 * ?	NoOperation;
 *
 * 1 - blockQuee
 */
block_state* getBlock(){
//	block_t* b = OS_MEMF_Alloc(&cmdPool);
	ktf_model.block_counter++;
	block_buffer_head = next_block_index(block_buffer_head);
//	if(block_buffer_head == block_buffer_tail){
//		NoOperation;
//		cmdQuee[block_buffer_head]=OS_MEMF_Alloc(&cmdPool,1);
//	}
//	else
#ifdef kTF7
		cmdQuee[block_buffer_head]=OS_MEMF_Alloc(&cmdPool,1);
#endif
//	if(cmdPool.pFree == (void*)1){
//		cmdPool.pFree = NULL;
//		NoOperation;
//	}
	return (cmdQuee[block_buffer_head]);
}


// from interrupt
void quee_blockRelease(){
//	uint8_t index = block_buffer_tail;
//	if(cmdPool.pFree == (void*)1)
//		NoOperation;
#ifdef KTF7
	if(OS_MEMF_GetNumFreeBlocks(&cmdPool)!=BLOCK_BUFFER_SIZE){
		OS_MEMF_Release(&cmdPool,cmdQuee[block_buffer_tail]);
		if (++block_buffer_tail == BLOCK_BUFFER_SIZE)
		{ block_buffer_tail = 0; }
	}
	else
		NoOperation;
#endif
}


block_state* plan_get_current_block(){
	block_state* result;
	next_buffer_head = next_block_index(next_buffer_head);
	result = cmdQuee[next_buffer_head];
	return(result);
}





