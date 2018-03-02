/*
 * cnc_sector.h
 *
 *  Created on: 24 авг. 2017 г.
 *      Author: walery
 */



#ifndef MSMOTOR_CNC_SECTOR_H_
#define MSMOTOR_CNC_SECTOR_H_

#include "msmotor/sSegment.h"

//--------------- function
extern void init_cncsector(void);

extern void buildSegment_MoveToXmin(struct sSegment* psc);


#endif /* MSMOTOR_CNC_SECTOR_H_ */
