/*
 * cnc_sector.c
 *
 *  Created on: 24 авг. 2017 г.
 *      Author: walery
 */

//--------------
#include "cnc_sector.h"

#include "msmotor/mempool.h"

#include "mSegmentQuee.h"

#include "msmotor/Microstep.h"
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
                psc = &sector[i];
//        psc = (struct sSegment*)MEMF_Alloc();
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

#define kalibrovka_INIT  354490//501325
#define kalibrovka_NORM  143781//203338

/**
 * #define DEFAULT_initial_rate    200530//141798//70898//50132
 * #define DEFAULT_nominal_rate    53528//37994//18997//13382//5370
 * #define DEFAULT_final_rate      200530//141798//70898//50132
 */
void buildSegment_MoveToXmin(struct sSegment* psc)
{
    struct sControl* pctl;

    NoOperation; //TODO buildSegment_startMoveToXmin
    psc->head.axis_number = 1;// Кол задействованных осей.
    psc->head.linenumber = 0x20;
    psc->head.axis_mask = X_FLAG;
    psc->head.reserved = 0x20;

    pctl = &psc->axis[X_AXIS];
    load_defaults(pctl);
    pctl->axis = X_AXIS;
    pctl->direction = backward;
    pctl->microsteps = Full_Step;
    pctl->initial_rate = kalibrovka_INIT;//258883;//517767;
    pctl->nominal_rate = kalibrovka_NORM;//105003;//81669;//163338;//81669;
    pctl->final_rate = pctl->initial_rate;
    pctl->steps = 100;  //TODO Параметры устройства.
    pctl->accelerate_until = 1;
    pctl->decelerate_after = pctl->steps - pctl->accelerate_until;
    //TODO buildSegment_startMoveToXmin

}

void buildSegment_MoveToXmax(struct sSegment* psc)
{
    struct sControl* pctl;

    NoOperation; //TODO buildSegment_startMoveToXmin
    psc->head.axis_number = 1;// Кол задействованных осей.
    psc->head.linenumber = 0x20;
    psc->head.axis_mask = X_FLAG;
    psc->head.reserved = 0x20;

    pctl = &psc->axis[X_AXIS];
    load_defaults(pctl);
    pctl->axis = X_AXIS;
    pctl->direction = forward; //backward
    pctl->microsteps = Full_Step;
    pctl->initial_rate = kalibrovka_INIT;//258883;//517767;
    pctl->nominal_rate = kalibrovka_NORM;//105003;//81669;//163338;//81669;
    pctl->final_rate = pctl->initial_rate;
    pctl->steps = 100;  //TODO Параметры устройства.
    pctl->accelerate_until = 1;
    pctl->decelerate_after = pctl->steps - pctl->accelerate_until;
    //TODO buildSegment_startMoveToXmin

}
