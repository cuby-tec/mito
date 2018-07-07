/*
 * status.c
 *
 *  Created on: 4 сент. 2017 г.
 *      Author: walery
 */

//--------------

#include "memf/mSegmentQuee.h"

#include "Status_t.h"
#include "hotend/hotendTask.h"
#include "msmotor/mempool.h"
#include "eModelstate.h"
#include "msmotor/ms_model.h"

//------------- defs

#define default_frameNumber  232
#define default_freeSegments  10
#define default_coordinatus  {123,234,345,456}
#define default_modelState  1   //eIdle
#define default_currentSegmentNumber  6
#define default_instrument2_parameter  246
#define default_instrument3_parameter  357
#define default_instrument4_parameter  468

//struct Status_t status;

//-------------- vars
const struct Status_t default_status = {default_frameNumber, default_freeSegments,
                                        default_coordinatus,
                                        default_modelState,
                                        default_currentSegmentNumber,
                                        default_instrument2_parameter,
                                        default_instrument3_parameter,
                                        default_instrument4_parameter

};

struct Status_t bu_status;

struct Status_t* ms_status = &bu_status;
//-------------- function

struct Status_t* getStatus(void)
{
    struct Status_t* result = ms_status;//&bu_status;
    static uint32_t frame_number = 0;

//    memcpy(result, &default_status, sizeof(struct Status_t));

//    result->freeSegments = MEMF_GetNumFreeBlocks();
    result->freeSegments = uxQueueSpacesAvailable( segmentQueue );
    result->currentSegmentNumber = getHeadLineNumber();
    result->frameNumber = ++frame_number;
    result->temperature = getCurrentHotendTemperature();
    result->modelState.queueState = uxQueueSpacesAvailable( segmentQueue );
//    result->modelState.modelState = pMs_State->instrumrnt1;

//    result->modelState.modelState = ehIdle;
//    result->coordinatus[X_AXIS] = current_pos[X_AXIS];
//    result->coordinatus[Y_AXIS] = current_pos[Y_AXIS];
//    result->coordinatus[Z_AXIS] = current_pos[Z_AXIS];
//    result->coordinatus[E_AXIS] = current_pos[E_AXIS];

    return result;
}

void init_Status()
{
//    memcpy(&bu_status, &default_status, sizeof(struct Status_t));
    bu_status.modelState.modelState = eIdle;
}
