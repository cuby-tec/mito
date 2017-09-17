/*
 * SegmentTask.h
 *
 *  Created on: 15 сент. 2017 г.
 *      Author: walery
 */

#ifndef MEMF_SEGMENTTASK_H_
#define MEMF_SEGMENTTASK_H_



//------------- defs
//#define SECTOR_TO_RELEAS     (1) // tail
#define SECTOR_RECEIVED     (1<<1)  // from Orderly


//-------------- vars

extern TaskHandle_t segmentHandling;

//--------------- function
extern uint32_t createSegmentTask(void);


#endif /* MEMF_SEGMENTTASK_H_ */
