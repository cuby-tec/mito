/*
 * Status_t.h
 *
 *  Created on: 4 сент. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_STATUS_T_H_
#define MSMOTOR_STATUS_T_H_

#include <stdint.h>
#include <stdbool.h>
#include "msmotor/msport.h"
#include "modelState.h"
#include "sModelCompState.h"

struct Status_t {
    uint32_t    frameNumber;
    uint32_t    freeSegments;   // Free segments value.
    int32_t    coordinatus[N_AXIS];
//    enum eModelState    modelState;
//    uint32_t    modelState;
    struct sModelCompState modelState;
    uint32_t    currentSegmentNumber;
    uint32_t    instrument2_parameter;
    uint32_t    instrument3_parameter;
    uint32_t    instrument4_parameter;
    // hotend section
    float       temperature;
};



#endif /* MSMOTOR_SSTATUS_T_H_ */
