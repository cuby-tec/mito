/*
 * svi_port.c
 *
 *  Created on: 15 авг. 2017 г.
 *      Author: walery
 */

//--------------

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"


#include "svi_port.h"
#include "msmotor/mempool.h"
#include "msmotor/msport.h"
#include "msmotor/ms_model.h"
//------------- defs


//-------------- vars
static uint32_t rest;
//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//-------------- function

// axis E -----------------------------------
void update_Eaxis(void)
{
            HWREGBITW(&g_ui32Flags,E_AXIS) ^= 1;
            GPIOPinWrite(DIRECTION_PORT, DIRECTION_E,(g_ui32Flags & E_FLAG)>>2);
    //    0x400253fc

            if(sts_e.counter > sts_e.point){
                 (*ms_finBlock)();
    //             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    //               return;
            }//else

            switch(pblock_e->schem[sts_e.state]){
            case 1:
                //      RISE_SPEED_FIRST;
                //      TIMER_Y += sts.rate_y;
                rest = 0;
                if(pblock_z->accelerate_until<=sts_e.counter){
                    sts_e.state++;
                    speedRate[Z_AXIS] = sts_e.rate;
                    sts_e.rate = pblock_z->nominal_rate;
                    sts_e.speedLevel = pblock_z->speedLevel;
                }else{
                    if(sts_e.speedLevel < pblock_z->speedLevel){
                        sts_e.speedLevel++;
                        if(sts_e.speedLevel == 1){
                            sts_e.rate *= 0.4056;
                        }
                        else{
    #ifdef DOUBLE
                            sts_e.rate = current_block->initial_rate*(sqrtf(sts_e.speedLevel+1)-sqrtf(sts_e.speedLevel));
    #else
                            sts_e.rate = sts_e.rate - (((2 * (long)sts_e.rate) + rest)/(4 * sts_e.speedLevel + 1));
    #endif
    //                    rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
                        //              sts.rate_y = (word)CO*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
                        }
                    }
                }
                break;

            case 2: case 5:// flast motion
                //      FLAT_MOTION;
                //      TIMER_Y += sts.rate_y;
                if(sts_e.counter>=pblock_z->decelerate_after){
                    sts_e.state++;
                    sts_e.rate = speedRate[Z_AXIS];
    #ifdef DOUBLE
                    sts_e.speedLevel--;
    #else
                    sts_e.speedLevel--;
    #endif
                    //          max_rate = sts.rate_y;
                }
                break;

            case 3:// decelerate
                //      DOWN_SPEED_LAST;
                rest = 2;
                sts_e.speedLevel--;
                if(sts_e.speedLevel>pblock_z->final_speedLevel){
    #ifdef DOUBLE
                    sts_e.rate = current_block->initial_rate*(sqrtf(sts_e.speedLevel+1)-sqrtf(sts_e.speedLevel));
    #else
    //                sts_e.rate_y = sts_e.rate_y + (((2 * (long)sts_e.rate_y) + rest)/(4 * sts_e.speedLevel + 1));
                    sts_e.rate = sts_e.rate + (((2 * (long)sts_e.rate))/(4 * sts_e.speedLevel + 1 + rest));
    //                rest = ((2 * (long)sts_e.rate_y)+rest)%(4 * sts_e.speedLevel + 1);
    #endif
                }else{
                    sts_e.rate = pblock_z->final_rate;
                }
                break;
            }

    //    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

        // end handler Y axis.



}


void update_Zaxis(void)
{
    //    uint32_t rest;
    //    g_ui32Flags |=
            HWREGBITW(&g_ui32Flags,Z_AXIS) ^= 1;
            GPIOPinWrite(DIRECTION_PORT, DIRECTION_Z,(g_ui32Flags & Z_FLAG)<<0);
    //    0x400253fc

            if(sts_z.counter > sts_z.point){
                 (*ms_finBlock)();
    //             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    //               return;
            }//else

            switch(pblock_e->schem[sts_z.state]){
            case 1:
                //      RISE_SPEED_FIRST;
                //      TIMER_Y += sts.rate_y;
                rest = 0;
                if(pblock_z->accelerate_until<=sts_z.counter){
                    sts_z.state++;
                    speedRate[Z_AXIS] = sts_z.rate;
                    sts_z.rate = pblock_z->nominal_rate;
                    sts_z.speedLevel = pblock_z->speedLevel;
                }else{
                    if(sts_z.speedLevel < pblock_z->speedLevel){
                        sts_z.speedLevel++;
                        if(sts_z.speedLevel == 1){
                            sts_z.rate *= 0.4056;
                        }
                        else{
    #ifdef DOUBLE
                            sts_z.rate = current_block->initial_rate*(sqrtf(sts_z.speedLevel+1)-sqrtf(sts_z.speedLevel));
    #else
                            sts_z.rate = sts_z.rate - (((2 * (long)sts_z.rate) + rest)/(4 * sts_z.speedLevel + 1));
    #endif
    //                    rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
                        //              sts.rate_y = (word)CO*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
                        }
                    }
                }
                break;

            case 2: case 5:// flast motion
                //      FLAT_MOTION;
                //      TIMER_Y += sts.rate_y;
                if(sts_z.counter>=pblock_z->decelerate_after){
                    sts_z.state++;
                    sts_z.rate = speedRate[Z_AXIS];
    #ifdef DOUBLE
                    sts_z.speedLevel--;
    #else
                    sts_z.speedLevel--;
    #endif
                    //          max_rate = sts.rate_y;
                }
                break;

            case 3:// decelerate
                //      DOWN_SPEED_LAST;
                rest = 2;
                sts_z.speedLevel--;
                if(sts_z.speedLevel>pblock_z->final_speedLevel){
    #ifdef DOUBLE
                    sts_z.rate = current_block->initial_rate*(sqrtf(sts_z.speedLevel+1)-sqrtf(sts_z.speedLevel));
    #else
    //                sts_z.rate_y = sts_z.rate_y + (((2 * (long)sts_z.rate_y) + rest)/(4 * sts_z.speedLevel + 1));
                    sts_z.rate = sts_z.rate + (((2 * (long)sts_z.rate))/(4 * sts_z.speedLevel + 1 + rest));
    //                rest = ((2 * (long)sts_z.rate_y)+rest)%(4 * sts_z.speedLevel + 1);
    #endif
                }else{
                    sts_z.rate = pblock_z->final_rate;
                }
                break;
            }

    //    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

        // end handler Y axis.


}

void update_Yaxis(void)
{
//    uint32_t rest;
    if(axis_flags&Y_FLAG)
    {
//            g_ui32Flags |=
        HWREGBITW(&g_ui32Flags,Y_AXIS) ^= 1;
        GPIOPinWrite(DIRECTION_PORT, DIRECTION_Y,(g_ui32Flags & Y_FLAG)<<2);
//            0x400253fc

        if(sts_y.counter > sts_y.point){
             (*ms_finBlock)();
//             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
//               return;
        }//else

        switch(pblock_e->schem[sts_y.state]){
        case 1:
            //      RISE_SPEED_FIRST;
            //      TIMER_Y += sts.rate_y;
            rest = 0;
            if(pblock_e->accelerate_until<=sts_y.counter){
                sts_y.state++;
                speedRate[Y_AXIS] = sts_y.rate;
                sts_y.rate = pblock_e->nominal_rate;
                sts_y.speedLevel = pblock_e->speedLevel;
            }else{
                if(sts_y.speedLevel < pblock_e->speedLevel){
                    sts_y.speedLevel++;
                    if(sts_y.speedLevel == 1){
                        sts_y.rate *= 0.4056;
                    }
                    else{
#ifdef DOUBLE
                        sts_y.rate = current_block->initial_rate*(sqrtf(sts_y.speedLevel+1)-sqrtf(sts_y.speedLevel));
#else
                        sts_y.rate = sts_y.rate - (((2 * (long)sts_y.rate) + rest)/(4 * sts_y.speedLevel + 1));
#endif
//                    rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
                    //              sts.rate_y = (word)CO*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
                    }
                }
            }
            break;

        case 2: case 5:// flast motion
            //      FLAT_MOTION;
            //      TIMER_Y += sts.rate_y;
            if(sts_y.counter>=pblock_e->decelerate_after){
                sts_y.state++;
//                if(sts.rate_y>psettings->initial_rate)
//                    sts.rate_y = psettings->initial_rate;
//                sts.rate_y = current_block->final_rate;
                sts_y.rate = speedRate[Y_AXIS];
#ifdef DOUBLE
                sts_y.speedLevel--;
#else
                sts_y.speedLevel--;
#endif
                //          max_rate = sts.rate_y;
            }
            break;

        case 3:// decelerate
            //      DOWN_SPEED_LAST;
            rest = 2;
            sts_y.speedLevel--;
            if(sts_y.speedLevel>pblock_e->final_speedLevel){
#ifdef DOUBLE
                sts_y.rate = pblock_e->initial_rate*(sqrtf(sts_y.speedLevel+1)-sqrtf(sts_y.speedLevel));
#else
//                sts_y.rate_y = sts_y.rate_y + (((2 * (long)sts_y.rate_y) + rest)/(4 * sts_y.speedLevel + 1));
                sts_y.rate = sts_y.rate + (((2 * (long)sts_y.rate))/(4 * sts_y.speedLevel + 1 + rest));
//                rest = ((2 * (long)sts_y.rate_y)+rest)%(4 * sts_y.speedLevel + 1);
#endif
            }else{
                sts_y.rate = pblock_e->final_rate;
            }
            break;
        }

//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

    }// end handler Y axis.

}

void update_Xaxis(void)
{

    // Обработка следующегошага.

    HWREGBITW(&g_ui32Flags,X_AXIS) ^= 1;
    GPIOPinWrite(DIRECTION_PORT, DIRECTION_X,(g_ui32Flags & X_FLAG)<<4);
    //            0x400253fc

    if(sts.counter > sts.point){
        (*ms_finBlock)();
        //             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
        //               return;
    }//else

    switch(pblock->schem[sts.state]){
    case 1:
        //      RISE_SPEED_FIRST;
        //      TIMER_Y += sts.rate_y;
        rest = 0;
        if(pblock->accelerate_until<=sts.counter){
            sts.state++;
            speedRate[X_AXIS] = sts.rate;
            sts.rate = pblock->nominal_rate;
            sts.speedLevel = pblock->speedLevel;
        }else{
            if(sts.speedLevel < pblock->speedLevel){
                sts.speedLevel++;
                if(sts.speedLevel == 1){
                    sts.rate *= 0.4056;
                }
                else{
#ifdef DOUBLE
sts.rate = pblock->initial_rate*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
#else
sts.rate = sts.rate - (((2 * (long)sts.rate) + rest)/(4 * sts.speedLevel + 1));
#endif
//                    rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
//              sts.rate_y = (word)CO*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
                }
            }
        }
        break;

    case 2: case 5:// flast motion
        //      FLAT_MOTION;
        //      TIMER_Y += sts.rate_y;
        if(sts.counter>=pblock->decelerate_after){
            sts.state++;
            //                if(sts.rate_y>psettings->initial_rate)
            //                    sts.rate_y = psettings->initial_rate;
            //                sts.rate_y = pblock->final_rate;
            sts.rate = speedRate[X_AXIS];
#ifdef DOUBLE
            sts.speedLevel--;
#else
            sts.speedLevel--;
#endif
            //          max_rate = sts.rate_y;
        }
        break;

    case 3:// decelerate
        //      DOWN_SPEED_LAST;
        rest = 2;
        sts.speedLevel--;
        if(sts.speedLevel>pblock->final_speedLevel){
#ifdef DOUBLE
            sts.rate = pblock->initial_rate*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
#else
            //                sts.rate_y = sts.rate_y + (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
            sts.rate = sts.rate + (((2 * (long)sts.rate))/(4 * sts.speedLevel + 1 + rest));
            //                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
#endif
        }else{
            sts.rate = pblock->final_rate;
        }
        break;
    }

    //    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

}

/**
 * Prepare data segment over USB.
 */
static uint32_t segment_request;

#define SET_SEGMENT_REQUEST(block, axis)    if(block.counter == block.point)\
        segment_request |= axis ## _FLAG

/**
 * Запросить новый сегмент (sSegment) блоков
 * из сектора(sSector) сегментов.
 */
void new_segment_request(void)
{
 // call segment USB.
}

/**
 * PA0 - interrupt vector used.
 */
void soft_interrupt_handler(void)
{
    uint32_t status = GPIOIntStatus(DIRECTION_PORT, false);
    if(axis_flags&X_FLAG)
    {
        update_Xaxis(); // port E pin 4
        axis_flags &= ~X_FLAG;
        SET_SEGMENT_REQUEST(sts,X);
    }

    if(axis_flags&Y_FLAG){
        update_Yaxis(); // port E pin 3
        axis_flags &= ~Y_FLAG;
        SET_SEGMENT_REQUEST(sts_y, Y);
    }

    if(axis_flags&Z_FLAG){
        update_Zaxis(); // port E pin 2
        axis_flags &= ~Z_FLAG;
        SET_SEGMENT_REQUEST(sts_z, Z);
    }

    if(axis_flags&E_FLAG){
        update_Eaxis();
        axis_flags &= ~E_FLAG;
        SET_SEGMENT_REQUEST(sts_e, E);
    }

//    if((segment_request&(X_FLAG)&sync[0])
//            && ( segment_request&(Y_FLAG)&sync[0] )
//            && ( segment_request&(Z_FLAG)&sync[0] )
//            && ( segment_request&(E_FLAG)&sync[0] ) )

    if(( segment_request & (X_FLAG|Y_FLAG|Z_FLAG|E_FLAG)) == sync[0])
    {
        new_segment_request();
    }

}




void svi_port_init(void)
{
    IntEnable(INT_GPIOA);
    IntPrioritySet(INT_GPIOA, SVI_PRIORITY);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_0);
    segment_request = 0;
    //
    // Trigger the INT_GPIOA interrupt.
    //
//    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.
}
