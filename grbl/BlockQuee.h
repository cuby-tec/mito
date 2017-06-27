/*
 * BlockQuee.h
 * Company: CUBY
 * Project: 052ktf
 *  Created on: 10.12.2012
 *      Author: walery
 */

#ifndef BLOCKQUEE_H_
#define BLOCKQUEE_H_

#include "Block_command.h"
#include "msmotor/mempool.h"
#include <stdint.h>

#include <msmotor/block_state.h>


//--- Defs


//---------------- Vars


extern volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
extern volatile uint8_t block_buffer_tail;       // Index of the block to process now
extern  uint8_t next_buffer_head;                 // Index of the next buffer head


//extern block_t* cmdQuee[BLOCK_BUFFER_SIZE];
extern block_state* cmdQuee[BLOCK_BUFFER_SIZE];

//----------------- Functions

//block_t* getBlock();
block_state* getBlock();
//struct block_t* getBlock();

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
uint8_t next_block_index(uint8_t block_index);

// Returns the index of the previous block in the ring buffer
uint8_t prev_block_index(uint8_t block_index);


//inline void plan_discard_current_block()
void plan_discard_current_block();

//inline block_t *plan_get_current_block()
//block_t *plan_get_current_block();
block_state *plan_get_current_block();

// Returns the availability status of the block ring buffer. True, if full.
uint8_t plan_check_full_buffer();

//  Освободить отработанный блок.
void quee_blockRelease();


#endif /* BLOCKQUEE_H_ */
