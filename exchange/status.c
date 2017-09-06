/*
 * status.c
 *
 *  Created on: 4 сент. 2017 г.
 *      Author: walery
 */

//--------------


#include "Status_t.h"

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

//-------------- function

struct Status_t* getStatus(void)
{
    return &default_status;
}

