/*
 * mSegmentQuee.c
 *
 *  Created on: 11 сент. 2017 г.
 *      Author: walery
 *  Description:     Операции с буфером Сегментов.
 */

//--------------

#include "inc/typedefs.h"
#include "msmotor/mempool.h"
#include "mSegmentQuee.h"
#include "cnc_sector.h"
#include "msmotor/ms_init.h"

//------------- defs

//-------------- vars
SemaphoreHandle_t memf_semaphor_handler;
static uint8_t semaphore_counter;

uint8_t next_buffer_head;  // Index of the next buffer head
uint8_t block_buffer_tail; // Index of the block to process now
uint8_t block_buffer_head; // Index of the next block to be pushed

//block_t* cmdQuee[BLOCK_BUFFER_SIZE];
struct sSegment* cmdQuee[SEGMENT_QUEE_SIZE];

//-------------- function
static uint8_t next_block_index(uint8_t block_index);
static uint8_t prev_block_index(uint8_t block_index);


struct sSegment* getSegment(void)
{
    struct sSegment* result = NULL;
    //cmdQuee[block_buffer_head]=OS_MEMF_Alloc(&cmdPool,1);
    if(xSemaphoreTake(memf_semaphor_handler,SEGMENT_DELAY))
    {
        /* The mutex was successfully obtained so the shared resource can be
         * accessed safely. */
        block_buffer_head = next_block_index(block_buffer_head);
        result = cmdQuee[block_buffer_head];

    }
    return (result);
}


struct sSegment* plan_get_current_block(void)
{
//    struct sSegment* result;
//    next_buffer_head = next_block_index(next_buffer_head);
    return (cmdQuee[block_buffer_tail]);
//    return(result);
}

// Освободить блок/Сегмент,т.е. увеличить сяётчик семафора,
//      передвинуть указатель block_buffer_head
uint8_t
memf_release(void)
{
    if(semaphore_counter<SEGMENT_QUEE_SIZE){
        if(xSemaphoreGive(memf_semaphor_handler) == pdPASS){
            semaphore_counter++;

            if (++block_buffer_tail == SEGMENT_QUEE_SIZE){
                block_buffer_tail = 0;
            }
            NoOperation;
        }
    }else{
        // Вызывающая задача должна бы перейти в состояние Suspend
        // , а это продолжение обработки прерывания
        // и для тестирования будем увеличивать счётчик.
        if(semaphore_counter<0xFF)
            semaphore_counter++;
        NoOperation;
    }
    return semaphore_counter;
}


 // Returns the index of the next block
static uint8_t
next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == SEGMENT_QUEE_SIZE) { block_index = 0; }
  return(block_index);
}

static init_cmdQuee(void){
    uint32_t i;

    initBlock();    // load defaults

    for(i=0;i<SEGMENT_QUEE_SIZE;i++){
        cmdQuee[i] = &sector[i];
    }
    block_buffer_tail = 0;
    block_buffer_head = block_buffer_tail;
    next_buffer_head  = block_buffer_tail;
}

// Returns the index of the previous block
static uint8_t
prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = SEGMENT_QUEE_SIZE; }
  block_index--;
  return(block_index);
}


void createSegmentQuee(void)
{
    // SemaphoreHandle_t SEGMENT_QUEE_SIZE
    memf_semaphor_handler = xSemaphoreCreateCounting(SEGMENT_QUEE_SIZE,SEGMENT_QUEE_SIZE);
    if(memf_semaphor_handler != NULL)
    {
        //TODO semaphore create
    /* The semaphore was created successfully. The semaphore can now be used. */

        semaphore_counter = SEGMENT_QUEE_SIZE;

        init_cmdQuee();


        NoOperation;
    }
}

