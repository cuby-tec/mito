/*
 * mSegmentQuee.h
 *
 *  Created on: 11 сент. 2017 г.
 *      Author: walery
 *      Операции с буфером Сегментов.
 */

#ifndef MEMF_MSEGMENTQUEE_H_
#define MEMF_MSEGMENTQUEE_H_


#include <FreeRTOS.h>
#include <semphr.h>
#include "msmotor/msport.h"

//------------- defs
#define MEMF_BIT0   (1<<0)

//-------------- vars
extern QueueHandle_t segmentQueueHandler;



extern SemaphoreHandle_t memf_semaphor_handler;

extern SemaphoreHandle_t rcvd_semaphore_handler;

//--------------- function

/**
 * Получить текущий рабочий Сегмент.
 */
extern struct sSegment* plan_get_current_block(void);

/**
 * Parameters: pmemf = null
 * pMemBlock = null
 */
//extern void memf_release(void* pmemf,void* pMemBlock);
// Возвращает значение счётчика, т.е. количество свободных мест в очереди сегментов.
extern uint8_t memf_release(void);

extern void createSegmentQuee(void);

/**
 * Return the pointer of the segment to load.
 * Сегмент полежащий загрузке.
 */
extern struct sSegment* getSegment(void);

extern uint32_t MEMF_GetNumFreeBlocks(void);

extern void* MEMF_Alloc(void);

/**
 * Номер последнего введённого Сегмента.
 */
extern uint32_t getHeadLineNumber(void);

/**
 * Переместить указатели текущего блока на следующий сегмент,
 * если он доступен.
 */
extern uint32_t move_pblock(void);

#endif /* MEMF_MSEGMENTQUEE_H_ */
