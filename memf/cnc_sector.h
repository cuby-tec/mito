/*
 * cnc_sector.h
 *
 *  Created on: 24 авг. 2017 г.
 *      Author: walery
 */



#ifndef MSMOTOR_CNC_SECTOR_H_
#define MSMOTOR_CNC_SECTOR_H_

#include "msmotor/sSegment.h"


//--------- defs

enum kl_move{
    kl_Xforward = 1,
    kl_Xbackward,
    kl_Yforward,
    kl_Ybackward
};


//--------------- function
extern void init_cncsector(void);

//        Движение в сторону оси Xmin; уменьшение координаты.
extern void buildSegment_MoveToXmin(struct sSegment* psc);

//        Движение в сторону оси Xmax; увеличение координаты.
extern void buildSegment_MoveToXmax(struct sSegment* psc);

extern void kl_buildSegment(struct sSegment* psc, enum kl_move axisdir);

#endif /* MSMOTOR_CNC_SECTOR_H_ */
