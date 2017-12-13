/*
 * cnc_sector.c
 *
 *  Created on: 24 авг. 2017 г.
 *      Author: walery
 */

//--------------

#include "msmotor/mempool.h"

#include "cnc_sector.h"
#include "mSegmentQuee.h"
//------------- defs

//-------------- vars


//-------------- function


static void load_defaults(struct sControl* pctl)
{
    pctl->steps = default_block.steps;
    pctl->microsteps = default_block.microsteps;
    pctl->accelerate_until = default_block.accelerate_until;
    pctl->decelerate_after = default_block.decelerate_after;
    pctl->initial_rate = default_block.initial_rate;
    pctl->initial_speedLevel = default_block.initial_speedLevel;
    pctl->nominal_rate = default_block.nominal_rate;
    pctl->speedLevel = default_block.speedLevel;
    pctl->final_rate = default_block.final_rate;
    pctl->final_speedLevel = default_block.final_speedLevel;
    pctl->schem[0] = default_block.schem[0];
    pctl->schem[1] = default_block.schem[1];
    pctl->schem[2] = default_block.schem[2];
    pctl->direction = default_block.direction;
}

void init_cncsector(void)
{
    uint32_t i,j;
    struct sSegment* psc;
    struct sControl* pctl;

    for(i=0;i<SEGMENT_QUEE_SIZE;i++)
    {
        //        sector[i].head.axis_number ;
        //        psc = &sector[i];
        psc = (struct sSegment*)MEMF_Alloc();
        if(psc){
            psc->head.axis_number = default_segment_head.axis_number; //N_AXIS;
            psc->head.linenumber = i + default_segment_head.linenumber;
            psc->head.axis_mask = default_segment_head.axis_mask;
            for(j=0;j<psc->head.axis_number;j++)
            {
                pctl = &psc->axis[j];
                switch(j){
                case X_AXIS:
                    load_defaults(pctl);
                    pctl->axis = X_AXIS;
                    break;
                case Y_AXIS:
                    load_defaults(pctl);
                    pctl->axis = Y_AXIS;
                    break;
                case Z_AXIS:
                    load_defaults(pctl);
                    pctl->axis = Z_AXIS;
                    break;
                case E_AXIS:
                    load_defaults(pctl);
                    pctl->axis = E_AXIS;
                    break;
                }
            }

            pctl = &psc->axis[X_AXIS];
            switch(i)
            {
            case 0:
                pctl->direction = backward;
                pctl->microsteps = 0;
                break;
            case 1:
                pctl->direction = forward;
                pctl->microsteps = 1;
                break;
            case 2:
                pctl->direction = backward;
                pctl->microsteps = 2;
            case 3:
                pctl->direction = forward;
                pctl->microsteps = 3;
                break;
            case 4:
                pctl->direction = backward;
                pctl->microsteps = 7;
                break;
            default:
                if(i % 2 == 0)
                    pctl->direction = backward;
                else
                    pctl->direction = forward;

                break;
            }
        }else{
            // Segment do not allocated.
            NoOperation;
        }

    }
}


