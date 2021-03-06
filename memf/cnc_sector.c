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
#define kalibrovka_accelerate_until 1

#define kalibrovka_INIT_Z1              183058
#define kalibrovka_NORM_Z1              39009
#define kalibrovka_accelerate_until_Z1  6

#define kalibrovka_INIT_Z2              141796
#define kalibrovka_NORM_Z2              30216
#define kalibrovka_accelerate_until_Z2  6

#define kalibrovka_INIT_Z3              129442
#define kalibrovka_NORM_Z3              27583
#define kalibrovka_accelerate_until_Z3  6

#define kalibrovka_INIT_Z4              112100
#define kalibrovka_NORM_Z4              23888
#define kalibrovka_accelerate_until_Z4  6

// 90
#define kalibrovka_INIT_Z5              105689
#define kalibrovka_NORM_Z5              22522
#define kalibrovka_accelerate_until_Z5  6

// 100
#define kalibrovka_INIT_Z6              100265
#define kalibrovka_NORM_Z6              21366
#define kalibrovka_accelerate_until_Z6  6

//150
#define kalibrovka_INIT_Z7              81866
#define kalibrovka_NORM_Z7              17445
#define kalibrovka_accelerate_until_Z7  6

// 200
#define kalibrovka_INIT_Z8              70898
#define kalibrovka_NORM_Z8              15108
#define kalibrovka_accelerate_until_Z8  6


#define kalibrovka_INIT_Z               kalibrovka_INIT_Z7
#define kalibrovka_NORM_Z               kalibrovka_NORM_Z7
#define kalibrovka_acelerate_until_Z    kalibrovka_accelerate_until_Z7


/**
 * #define DEFAULT_initial_rate    200530//141798//70898//50132
 * #define DEFAULT_nominal_rate    53528//37994//18997//13382//5370
 * #define DEFAULT_final_rate      200530//141798//70898//50132
 */
void buildSegment_MoveToXmin(struct sSegment* psc)
{
    struct sControl* pctl;

    NoOperation; // buildSegment_startMoveToXmin
    psc->head.axis_number = 1;// Кол задействованных осей.
    psc->head.linenumber = 0x20;
    psc->head.axis_mask = X_FLAG;
    psc->head.reserved = 0x20;
    psc->head.reserved &= ~EXIT_CONTINUE; // stop after execution.

    pctl = &psc->axis[X_AXIS];
    load_defaults(pctl);
    pctl->axis = X_AXIS;
    pctl->direction = backward;
    pctl->microsteps = Full_Step;
    pctl->initial_rate = kalibrovka_INIT;//258883;//517767;
    pctl->nominal_rate = kalibrovka_NORM;//105003;//81669;//163338;//81669;
    pctl->final_rate = pctl->initial_rate;
    pctl->steps = 100;  //TODO Параметры устройства.
    pctl->accelerate_until = kalibrovka_accelerate_until;//1;
    pctl->decelerate_after = pctl->steps - pctl->accelerate_until;

}

void buildSegment_MoveToXmax(struct sSegment* psc)
{
    struct sControl* pctl;

    NoOperation; //buildSegment_startMoveToXmin
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
    pctl->accelerate_until = kalibrovka_accelerate_until;//1;
    pctl->decelerate_after = pctl->steps - pctl->accelerate_until;

}



void kl_buildSegment(struct sSegment* psc, enum kl_move axisdir)
{
    struct sControl* pctl;

    switch(axisdir){
    case kl_Xforward:
//        NoOperation; //buildSegment_startMoveToXmin
        psc->head.axis_number = 1;// Кол задействованных осей.
        psc->head.linenumber = 0x20;
        psc->head.axis_mask = X_FLAG;
//        psc->head.reserved = 0x20;

        pctl = &psc->axis[X_AXIS];
        load_defaults(pctl);
        pctl->axis = X_AXIS;
        pctl->direction = forward; //backward
        pctl->microsteps = Full_Step;
        pctl->initial_rate = kalibrovka_INIT;//258883;//517767;
        pctl->nominal_rate = kalibrovka_NORM;//105003;//81669;//163338;//81669;
        pctl->final_rate = pctl->initial_rate;
        pctl->steps = 100;  //TODO Параметры устройства.
        pctl->accelerate_until = kalibrovka_accelerate_until;//1;
        pctl->decelerate_after = pctl->steps - pctl->accelerate_until;

        break;

    case kl_Xbackward:
        psc->head.axis_number = 1;// Кол задействованных осей.
        psc->head.linenumber = 0x21;
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
        pctl->accelerate_until = kalibrovka_accelerate_until;//1;
        pctl->decelerate_after = pctl->steps - pctl->accelerate_until;

        break;

    case kl_Yforward:
        psc->head.axis_number = 1;// Кол задействованных осей.
        psc->head.linenumber = 0x23;
        psc->head.axis_mask = Y_FLAG;
        psc->head.reserved = 0x23;

        pctl = &psc->axis[Y_AXIS];
        load_defaults(pctl);
        pctl->axis = Y_AXIS;
        pctl->direction = forward;
        pctl->steps = 100;  //TODO Параметры устройства.
        pctl->microsteps = Full_Step;
        pctl->initial_rate = kalibrovka_INIT;//258883;//517767;
        pctl->nominal_rate = kalibrovka_NORM;//105003;//81669;//163338;//81669;
        pctl->final_rate = pctl->initial_rate;
        pctl->accelerate_until = kalibrovka_accelerate_until;//1;
        pctl->decelerate_after = pctl->steps - pctl->accelerate_until;
        break;

    case kl_Ybackward:
        psc->head.axis_number = 1;// Кол задействованных осей.
        psc->head.linenumber = 0x22;
        psc->head.axis_mask = Y_FLAG;
        psc->head.reserved = 0x22;

        pctl = &psc->axis[Y_AXIS];
        load_defaults(pctl);
        pctl->axis = Y_AXIS;
        pctl->direction = backward;
        pctl->steps = 100;  //TODO Параметры устройства.
        pctl->microsteps = Full_Step;
        pctl->initial_rate = 200530;//kalibrovka_INIT;//258883;//517767;
        pctl->nominal_rate = 81335;//kalibrovka_NORM;//105003;//81669;//163338;//81669;
        pctl->final_rate = pctl->initial_rate;
        pctl->accelerate_until = kalibrovka_accelerate_until;//1;
        pctl->decelerate_after = pctl->steps - pctl->accelerate_until;
        break;

    case kl_Zforward:
        psc->head.axis_number = 1;// Кол задействованных осей.
        psc->head.linenumber = 0x24;
        psc->head.axis_mask = Z_FLAG;
        psc->head.reserved = 0x24;

        pctl = &psc->axis[Z_AXIS];
        load_defaults(pctl);
        pctl->axis = Z_AXIS;
        pctl->direction = forward;
        pctl->steps = 400;  //TODO Параметры устройства.
        pctl->microsteps = Full_Step;
        pctl->initial_rate = kalibrovka_INIT_Z;//258883;//517767;
        pctl->nominal_rate = kalibrovka_NORM_Z;//105003;//81669;//163338;//81669;
        pctl->final_rate = pctl->initial_rate;
        pctl->accelerate_until = kalibrovka_acelerate_until_Z;//kalibrovka_accelerate_until;//1;
        pctl->decelerate_after = pctl->steps - pctl->accelerate_until;
        break;

    case kl_Zbackward:
        psc->head.axis_number = 1;// Кол задействованных осей.
        psc->head.linenumber = 0x25;
        psc->head.axis_mask = Z_FLAG;
        psc->head.reserved = 0x25;

        pctl = &psc->axis[Z_AXIS];
        load_defaults(pctl);
        pctl->axis = Z_AXIS;
        pctl->direction = backward;
        pctl->steps = 400;  //TODO Параметры устройства.
        pctl->microsteps = Full_Step;
        pctl->initial_rate = kalibrovka_INIT_Z;//258883;//517767;
        pctl->nominal_rate = kalibrovka_NORM_Z;//105003;//81669;//163338;//81669;
        pctl->final_rate = pctl->initial_rate;
        pctl->accelerate_until = kalibrovka_acelerate_until_Z;//kalibrovka_accelerate_until;//1;
        pctl->decelerate_after = pctl->steps - pctl->accelerate_until;

        break;


    }// end switch(axisdir)
}


