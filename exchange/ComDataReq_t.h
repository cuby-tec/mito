/*
 * ComDataReq_t.h
 *
 *  Created on: 10 сент. 2017 г.
 *      Author: walery
 */

#ifndef EXCHANGE_COMDATAREQ_T_H_
#define EXCHANGE_COMDATAREQ_T_H_

#include "msmotor/sSegment.h"

//------------- defs

struct ComDataReq_t{
    uint32_t    requestNumber;
    uint8_t     size;
    uint8_t     instruments;
    uint16_t    command;
    struct sSegment instrument1_paramter;
    uint32_t    instrument2_paramter;
    uint32_t    instrument3_paramter;
    uint32_t    instrument4_paramter;
};


//-------------- vars


//--------------- function



#endif /* EXCHANGE_COMDATAREQ_T_H_ */
