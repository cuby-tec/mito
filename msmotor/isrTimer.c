/*
 * isrTimer_X.c
 *
 *  Created on: 1 июл. 2017 г.
 *      Author: walery
 */

//--------------



#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#ifdef TARGET_IS_TM4C123_RA1
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#endif

#include "msport.h"
#include "msmotor/isrTimer.h"
#include "mempool.h"
#include "ms_model.h"

//------------- defs

#define update_currentPos(axes)      if(segment->axis[axes].direction == forward)\
                current_pos[axes]++;\
            else\
                current_pos[axes]--
//-------------- vars
static uint32_t multy = 1;

//-------------- function

#ifdef commit_13
void Timer_X_isr(void){
    static uint32_t cnt_x;
    TimerIntClear(TIMER_BASE_X_AXIS,TIMER_CAPA_EVENT);//TIMER_TIMB_TIMEOUT CAEIM
    cnt_x++;
    if(cnt_x & 1){
        GPIOPinWrite(GPIO_PORTF_BASE, RED_GPIO_PIN, RED_GPIO_PIN);
//        xTaskNotifyFromISR(orderlyHandling,0x01,eSetBits,NULL);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE, RED_GPIO_PIN, ~RED_GPIO_PIN);
//        xTaskNotifyFromISR(orderlyHandling,0x02,eSetBits,NULL);
    }
}
#endif

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
// X axis.
//
//*****************************************************************************
void
Timer1IntHandler(void)
{
    //    uint32_t cOne = 1, cTwo;

    uint32_t timerValue_X;

    TimerIntClear(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);
    //
/*    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_GPIO_PIN, GPIO_PIN_2);*/

    sts.counter++;

    if(sts.counter>sts.point){
        HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    }else{
        timerValue_X = sts.rate*multy;

        TimerLoadSet(TIMER_BASE_X_AXIS,TIMER_X,timerValue_X);
        timerValue_X >>=16;
        TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_X, (uint16_t)(timerValue_X&0x000000FF));
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAPR) = (uint16_t)(uint16_t)(timerValue&0x000000FF);

    }
    axis_flags |= X_FLAG;
/*
    GPIOPinWrite(GPIO_PORTF_BASE, BLUE_GPIO_PIN, ~GPIO_PIN_2);
    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.
    //           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
*/
//==============
    // Обработка следующего шага.

    if(sts.counter > sts.point){
        mask_axis[0] |= pblock->axis;//X_FLAG;
        (*ms_finBlock)();
    }else
        update_currentPos(X_AXIS);

    switch(pblock->schem[sts.state]){
    case 1:
        //      RISE_SPEED_FIRST;
//        rest = 0;
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
 sts.rate = sts.rate - (((2 * (long)sts.rate))/(4 * sts.speedLevel + 1));
#endif
                }
            }
        }
        break;

    case 2: case 5:// flast motion
        //      FLAT_MOTION;
        //      TIMER_Y += sts.rate_y;
        if(sts.counter>=pblock->decelerate_after){
            sts.state++;
            sts.rate = speedRate[X_AXIS];
#ifdef DOUBLE
            sts.speedLevel--;
#else
            sts.speedLevel--;
#endif
        }
        break;

    case 3:// decelerate
        //      DOWN_SPEED_LAST;
        sts.speedLevel--;
        if(sts.speedLevel>pblock->final_speedLevel){
#ifdef DOUBLE
            sts.rate = pblock->initial_rate*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
#else
            sts.rate = sts.rate + (((2 * (long)sts.rate))/(4 * sts.speedLevel + 1));
#endif
        }else{
            sts.rate = pblock->final_rate;
        }
        break;
    }

//=============
    return;
}

void TimerEIntHandler(void)
{
    uint32_t timerValue_E;

    // Clear the timer interrupt.
    TimerIntClear(TIMER_BASE_E_AXIS, TIMER_CAPB_EVENT);

    sts_e.counter++;

    if(sts_e.counter>sts_e.point){
        HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
    }else{
        timerValue_E = sts_e.rate*multy;

        TimerLoadSet(TIMER_BASE_E_AXIS, TIMER_E, timerValue_E);
        timerValue_E >>=16;
        TimerPrescaleSet(TIMER_BASE_E_AXIS, TIMER_E, (uint16_t)(timerValue_E&0x000000FF));
    }
    axis_flags |= E_FLAG;
/*    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.*/
//=================
    if(sts_e.counter > sts_e.point){
        mask_axis[0] |= pblock_e->axis;//X_FLAG;
         (*ms_finBlock)();
    }else
        update_currentPos(E_AXIS);

    switch(pblock_e->schem[sts_e.state]){
    case 1:
        //      RISE_SPEED_FIRST;
        //      TIMER_Y += sts.rate_y;
        if(pblock_e->accelerate_until<=sts_e.counter){
            sts_e.state++;
            speedRate[Z_AXIS] = sts_e.rate;
            sts_e.rate = pblock_e->nominal_rate;
            sts_e.speedLevel = pblock_e->speedLevel;
        }else{
            if(sts_e.speedLevel < pblock_e->speedLevel){
                sts_e.speedLevel++;
                if(sts_e.speedLevel == 1){
                    sts_e.rate *= 0.4056;
                }
                else{
#ifdef DOUBLE
                    sts_e.rate = current_block->initial_rate*(sqrtf(sts_e.speedLevel+1)-sqrtf(sts_e.speedLevel));
#else
                    sts_e.rate = sts_e.rate - ((2 * (long)sts_e.rate)/(4 * sts_e.speedLevel + 1));
#endif
                }
            }
        }
        break;

    case 2: case 5:// flast motion
        //      FLAT_MOTION;
        //      TIMER_Y += sts.rate_y;
        if(sts_e.counter>=pblock_e->decelerate_after){
            sts_e.state++;
            sts_e.rate = speedRate[Z_AXIS];
#ifdef DOUBLE
            sts_e.speedLevel--;
#else
            sts_e.speedLevel--;
#endif
        }
        break;

    case 3:// decelerate
        //      DOWN_SPEED_LAST;
        sts_e.speedLevel--;
        if(sts_e.speedLevel>pblock_e->final_speedLevel){
#ifdef DOUBLE
            sts_e.rate = current_block->initial_rate*(sqrtf(sts_e.speedLevel+1)-sqrtf(sts_e.speedLevel));
#else
            sts_e.rate = sts_e.rate + (((2 * (long)sts_e.rate))/(4 * sts_e.speedLevel + 1 ));
#endif
        }else{
            sts_e.rate = pblock_e->final_rate;
        }
        break;
    }

//=================
    return;
}

//  Axis Y handler
void TimerYIntHandler(void)
{
    //    char cOne, cTwo;
    uint32_t timerValue_Y;

    // Clear the timer interrupt.
    TimerIntClear(TIMER_BASE_Y_AXIS, TIMER_CAPB_EVENT);

    sts_y.counter++;

    if(sts_y.counter>sts_y.point){
        HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
    }else{
        timerValue_Y = sts_y.rate*multy;

        TimerLoadSet(TIMER_BASE_Y_AXIS, TIMER_Y, timerValue_Y);
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = sts.rate_y*multy;
        //           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = (uint16_t)(timerValue&0x0000FFFF);
        timerValue_Y >>=16;
        TimerPrescaleSet(TIMER_BASE_Y_AXIS, TIMER_Y, (uint16_t)(timerValue_Y&0x000000FF));
    }
//    axis_flags |= Y_FLAG;
/*    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.*/
//===================
    if(sts_y.counter > sts_y.point){
        mask_axis[0] |= pblock_y->axis;//X_FLAG;
        (*ms_finBlock)();
    }else
        update_currentPos(Y_AXIS);

    switch(pblock_y->schem[sts_y.state]){
    case 1:
        //      RISE_SPEED_FIRST;
        //      TIMER_Y += sts.rate_y;
        if(pblock_y->accelerate_until<=sts_y.counter){
            sts_y.state++;
            speedRate[Y_AXIS] = sts_y.rate;
            sts_y.rate = pblock_y->nominal_rate;
            sts_y.speedLevel = pblock_y->speedLevel;
        }else{
            if(sts_y.speedLevel < pblock_y->speedLevel){
                sts_y.speedLevel++;
                if(sts_y.speedLevel == 1){
                    sts_y.rate *= 0.4056;
                }
                else{
#ifdef DOUBLE
                    sts_y.rate = current_block->initial_rate*(sqrtf(sts_y.speedLevel+1)-sqrtf(sts_y.speedLevel));
#else
                    sts_y.rate = sts_y.rate - ((2 * (long)sts_y.rate )/(4 * sts_y.speedLevel + 1));
#endif
                }
            }
        }
        break;

    case 2: case 5:// flast motion
        //      FLAT_MOTION;
        //      TIMER_Y += sts.rate_y;
        if(sts_y.counter>=pblock_y->decelerate_after){
            sts_y.state++;
            sts_y.rate = speedRate[Y_AXIS];
#ifdef DOUBLE
            sts_y.speedLevel--;
#else
            sts_y.speedLevel--;
#endif
        }
        break;

    case 3:// decelerate
        //      DOWN_SPEED_LAST;
        sts_y.speedLevel--;
        if(sts_y.speedLevel>pblock_y->final_speedLevel){
#ifdef DOUBLE
            sts_y.rate = pblock_y->initial_rate*(sqrtf(sts_y.speedLevel+1)-sqrtf(sts_y.speedLevel));
#else
//                sts_y.rate_y = sts_y.rate_y + (((2 * (long)sts_y.rate_y) + rest)/(4 * sts_y.speedLevel + 1));
            sts_y.rate = sts_y.rate + (((2 * (long)sts_y.rate))/(4 * sts_y.speedLevel + 1 ));
//                rest = ((2 * (long)sts_y.rate_y)+rest)%(4 * sts_y.speedLevel + 1);
#endif
        }else{
            sts_y.rate = pblock_y->final_rate;
        }
        break;
    }
//===================
    return;
}

// Axis Z handler -------------------------------
void TimerZIntHandler(void)
{
    uint32_t timerValue_Z;

    // Clear the timer interrupt.
    TimerIntClear(TIMER_BASE_Z_AXIS, TIMER_CAPA_EVENT);

    sts_z.counter++;

    if(sts_z.counter>sts_z.point){
        HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    }else{
        timerValue_Z = sts_z.rate*multy;

        TimerLoadSet(TIMER_BASE_Z_AXIS, TIMER_Z, timerValue_Z);
        timerValue_Z >>=16;
        TimerPrescaleSet(TIMER_BASE_Z_AXIS, TIMER_Z, (uint16_t)(timerValue_Z&0x000000FF));
    }
    axis_flags |= Z_FLAG;
/*    HWREG(NVIC_SW_TRIG) = INT_GPIOA - 16; //Trigger the INT_GPIOA interrupt.*/

//===================
    if(sts_z.counter > sts_z.point){
        mask_axis[0] |= pblock_z->axis;//X_FLAG;
        (*ms_finBlock)();
    }else
        update_currentPos(Z_AXIS);

    switch(pblock_z->schem[sts_z.state]){
    case 1:
        //      RISE_SPEED_FIRST;
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
                    sts_z.rate = sts_z.rate - ((2 * (long)sts_z.rate )/(4 * sts_z.speedLevel + 1));
#endif
                }
            }
        }
        break;

    case 2: case 5:// flast motion
        //      FLAT_MOTION;
        if(sts_z.counter>=pblock_z->decelerate_after){
            sts_z.state++;
            sts_z.rate = speedRate[Z_AXIS];
#ifdef DOUBLE
            sts_z.speedLevel--;
#else
            sts_z.speedLevel--;
#endif
        }
        break;

    case 3:// decelerate
        //      DOWN_SPEED_LAST;
        sts_z.speedLevel--;
        if(sts_z.speedLevel>pblock_z->final_speedLevel){
#ifdef DOUBLE
            sts_z.rate = current_block->initial_rate*(sqrtf(sts_z.speedLevel+1)-sqrtf(sts_z.speedLevel));
#else
            sts_z.rate = sts_z.rate + (((2 * (long)sts_z.rate))/(4 * sts_z.speedLevel + 1 ));
#endif
        }else{
            sts_z.rate = pblock_z->final_rate;
        }
        break;
    }

//===================
    return;
}

