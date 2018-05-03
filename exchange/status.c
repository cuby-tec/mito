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
uint32_t frame_number = 0;

//-------------- function

struct Status_t* getStatus(void)
{
    struct Status_t* result = &bu_status;

//    memcpy(result, &default_status, sizeof(struct Status_t));

    result->freeSegments = MEMF_GetNumFreeBlocks();
    result->currentSegmentNumber = getHeadLineNumber();
    result->frameNumber = ++frame_number;
    result->temperature = getCurrentHotendTemperature();

    return result;
}

void init_Status()
{
    memcpy(&bu_status, &default_status, sizeof(struct Status_t));
}
