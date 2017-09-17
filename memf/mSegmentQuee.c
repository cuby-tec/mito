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
SemaphoreHandle_t rcvd_semaphore_handler;

static uint8_t semaphore_counter;

uint8_t next_buffer_head;  // Index of the next buffer head
uint8_t block_buffer_tail; // Index of the block to process now
uint8_t block_buffer_head; // Index of the next block to be pushed

//block_t* cmdQuee[BLOCK_BUFFER_SIZE];
struct sSegment* cmdQuee[SEGMENT_QUEE_SIZE];

//-------------- function
static uint8_t next_block_index(uint8_t block_index);
static uint8_t prev_block_index(uint8_t block_index);


uint32_t
MEMF_GetNumFreeBlocks(void)
{
    uint32_t result = 0;
    if(block_buffer_head<block_buffer_tail){
        result += SEGMENT_QUEE_SIZE;
    }
    return (result+block_buffer_head - block_buffer_tail);
}

uint32_t
getHeadLineNumber(void)
{
    uint32_t result;
    struct sSegment* segment = cmdQuee[block_buffer_head];

    if(segment != NULL)
        //        return (cmdQuee[block_buffer_head]->head.linenumber);
        result = segment->head.linenumber;

    return (result);
}

static init_next_block(uint32_t index){
    struct sSegment* segment = cmdQuee[index];

    pblock = &segment->axis[X_AXIS];
    pblock_y = &segment->axis[Y_AXIS];
    pblock_z = &segment->axis[Z_AXIS];
    pblock_e = &segment->axis[E_AXIS];
}

/**
 * Переместить указатели текущего блока на следующий сегмент,
 * если он доступен.
 */
uint32_t
move_pblock(void)
{
    uint32_t result, next_segment;

    next_segment = next_block_index(block_buffer_tail);
    if(next_segment != next_block_index(block_buffer_head)){
        // сегмент next_segment актуален.
        init_next_block(next_segment);
        result = TRUE;
    }else{
        result = FALSE;
    }

    return (result);
}

/**
 * Выделение места для нового блока MEMF.
 * если места нет, то ожидать семафора.
 */
void* MEMF_Alloc(void)
{
    BaseType_t sem;

    sem = xSemaphoreTake(memf_semaphor_handler, portMAX_DELAY);
    if(sem == pdTRUE){
        block_buffer_head = next_block_index(block_buffer_head);
        return (void*)cmdQuee[block_buffer_head];
    }
}

/**
 * Смещение head для записи нового сегмента.
 */
struct sSegment* getSegment(void)
{
    struct sSegment* result = NULL;
    //cmdQuee[block_buffer_head]=OS_MEMF_Alloc(&cmdPool,1);
//    if(xSemaphoreTake(memf_semaphor_handler,SEGMENT_DELAY))
//    {
        /* The mutex was successfully obtained so the shared resource can be
         * accessed safely. */
        block_buffer_head = next_block_index(block_buffer_head);
        result = cmdQuee[block_buffer_head];

//    }
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
memf_release(void){
    //    if(semaphore_counter<SEGMENT_QUEE_SIZE){
    if(xSemaphoreGive(memf_semaphor_handler) == pdPASS){
        semaphore_counter--;

        block_buffer_tail = next_block_index(block_buffer_tail);
        NoOperation;
        //    }
    }else{
        // Вызывающая задача должна бы перейти в состояние Suspend
        // , а это продолжение обработки прерывания
        // и для тестирования будем увеличивать счётчик.
        //        if(semaphore_counter<0xFF)
        //            semaphore_counter++;
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

    block_buffer_tail = 0;
    block_buffer_head = block_buffer_tail;
    next_buffer_head  = block_buffer_tail;

   for(i=0;i<SEGMENT_QUEE_SIZE;i++){
        cmdQuee[i] = &sector[i];
        block_buffer_head = i;
    }
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
        rcvd_semaphore_handler = xSemaphoreCreateMutex();
        if(rcvd_semaphore_handler == NULL){
            while(1)
            {
                /* There was insufficient heap memory available for the mutex to be created. */
                NoOperation;
            }
        }
    }
}

