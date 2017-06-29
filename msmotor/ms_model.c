/*
 * ms_model.c
 *
 *  Created on: 21 июн. 2017 г.
 *      Author: walery
 */



//--------------
#include <stdlib.h>
#include "inc/sysDrivers.h"
//#include <sysbiosHeader.h>

#include "inc/typedefs.h"
#include <grbl/settings.h>

#include "msmotor/block_state.h"
#include "ms_model.h"
#include "msport.h"


#include "Ktf_model.h"
#include "msmotor/mem_flash.h"
#include "grbl/planner.h"

//------------- defs

#define PE0     GPIO_PIN_0
#define PE1     GPIO_PIN_1
#define PE2     GPIO_PIN_2
#define PE3     GPIO_PIN_3
#define PE4     GPIO_PIN_4



#define Y_STEP_BIT          7  // Port 5 pin 7
#define Y_DIRECTION_BIT     6  // Port 5 Pin 6
#define STEP_MASK           (1<<Y_STEP_BIT) // All step bits
#define DIRECTION_MASK      (1<<Y_DIRECTION_BIT) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)



//-------------- vars
block_state* current_block;

stepper_state sts;

//static float start_speed = 0.0066;
byte direction = false;

static void (*ms_finBlock)(void);



//-------------- function


/**
 *  button control
 *  Синхронная работа считывателя блоков.
*/
static void ms_nextBlock(){
//  MS_ENABLE_OUT = MS_STOP; // Pause
//    if(ktf_model.block_counter == block_target){
//        NoOperation;
//    }

//    MS_COMMAND_SEMA_IE; // Ожидает достижения координаты X
//    STOP_Y;
    NoOperation;
}

/**
 * Асинхронная работа считывателя блоков.
 */
static void ms_async_block(){
//    MS_ENABLE_OUT = MS_STOP; // Pause
//    MS_COMMAND_SEMA_SET;
    NoOperation;
}



static word path_length = 2000;

void testPrepare(void){
    //  current_block setup
#ifdef KTF7
    current_block = (block_state*)OS_malloc(sizeof(block_state));
#else
    current_block = (block_state*)malloc(sizeof(block_state));
#endif
    if(current_block){
        memset(current_block,0,sizeof(block_state));
//      current_block->initial_rate = start_c;
        current_block->steps_y = path_length;
        current_block->step_event_count = current_block->steps_y;
#ifdef KTF7
        current_block->direction_bits |= STEP_MASK|DIRECTION_MASK;
#else
        current_block->direction_bits |= PE0|PE1|PE2|PE3|PE4; //DIRECTION_MASK;
#endif
//      current_block->nominal_rate = 4000;
//      current_block->final_rate = 40000;
//      current_block->nominal_speed = 6.6;
        current_block->schem[0] = 10;
        current_block->schem[1] = 11;
        current_block->schem[2] = 12;
        current_block->state = 0;
//      current_block->tan_theta = current_block->steps_y/current_block->steps_x;
        // #define INITIAL_RATE             DEFAULT_COUNTERR_FREWUENCY*kalf*sqrt(2*alfa/psettings->rad_acceleration)//20000 //  Определяет ускорение разгона.
        double_t dt = INITIAL_RATE;
//      double_t dq = alfa;
//      dq *= 2;
//      dq /= psettings->rad_acceleration;
//      dq = sqrt(dq);
//      dt *= kalf;
//      dt *= dq;
        if(dt>0xffff)
//          psettings->initial_rate = 0xFFFF;
            ktf_model.initial_rate = 0xFFFF;
        else
//          psettings->initial_rate = dt;
            ktf_model.initial_rate = dt;

        setSpeedLevel(current_block,psettings->seekSpeed);
//      current_block->accelerate_until = speed_rate;
        current_block->accelerate_until = current_block->speedLevel;
        current_block->decelerate_after = current_block->step_event_count - current_block->accelerate_until;
        //  sts setup --------------------------------------
        sts.counter_y = 0; //1;
        sts.counter_x = 0;
        sts.speedLevel = 0;
        //          sts.rate_delta_y = current_block->rate_delta;
        sts.rate_y = current_block->initial_rate;

        sts.point_y = current_block->steps_y;

//      if(current_block->direction_bits&DIRECTION_MASK)
//          STEPPING_PORT |= DIRECTION_MASK;
//      else
//          STEPPING_PORT &=~DIRECTION_MASK;
#if !defined TM4
        if(direction){
            STEPPING_PORT |= DIRECTION_MASK;
            current_block->direction_bits |=DIRECTION_MASK;
        }else{
            STEPPING_PORT &=~DIRECTION_MASK;
            current_block->direction_bits &=~DIRECTION_MASK;
        }
        direction = ~direction;
#endif
    }else{
        NoOperation;
    }



    // Задание режима обработки блоков движения.
    switch(ktf_model.sync){
    case eSyncMode_ms:
        ms_finBlock = ms_nextBlock;     //Синхронный режим
        break;
    case eAsyncMode_ms:
        ms_finBlock = ms_async_block;   // Асинхронный режим.
        break;
    }

//    TimerEnable(TIMER0_BASE, TIMER_B);
    TimerEnable(TIMER1_BASE, TIMER_BOTH);

//  START_Y;

}// end void


uint32_t cnt_int = 0;
//------------- interrupt

// Keep track of remainder from new_step-delay calculation to increase accuracy
static unsigned int rest = 0;
#define DEBUG_INT_no
/**
 * Обработчик прерываний оси X
 */
void axisX_intrrupt_handler(void){

    TimerIntClear(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_ICR) |= TIMER_ICR_CAECINT;

#ifdef DEBUG_INT
//    uint32_t val;
    cnt_int++;
        System_printf("axisX_intrrupt_handler cnt-int:%lu\n",cnt_int);
        System_flush();
//        val = TimerValueGet(TIMER0_BASE, TIMER_B);
//        System_printf("Timer value %lu \n",val);
#else

//    OS_EnterInterrupt();
//    STEPPING_PORT |= (STEP_MASK);
//    if(STEP_MASK&current_block->direction_bits){
        if(current_block->direction_bits & PE0){
            _posY++;
        }
        else{
            _posY--;
        }

        //      switch(current_block->schem[current_block->state]){
//    }
//    STEPPING_PORT &= ~STEP_MASK;
    sts.counter_y++;

//    TIMER_Y += sts.rate_y;
    if(sts.rate_y>0x4f){
        TimerLoadSet(TIMER0_BASE, TIMER_B, sts.rate_y);
//        System_printf("Load timer 0_B .\n");
    }
    else{
//        System_printf("Error:axisX_intrrupt_handler: sts.rate_y = %lu  \n",sts.rate_y);
        while(1);
    }


    if(sts.counter_y>=sts.point_y){
        (*ms_finBlock)();
    }else


    //      switch(sts.state){
        switch(current_block->schem[current_block->state]){
        case 1:
            //      RISE_SPEED_FIRST;
            //      TIMER_Y += sts.rate_y;
            if(current_block->accelerate_until<=sts.counter_y){
                current_block->state++;
                sts.rate_y = current_block->nominal_rate;
                sts.speedLevel = current_block->speedLevel;
            }else{
                if(sts.speedLevel < current_block->speedLevel){
                    sts.speedLevel++;
                    sts.rate_y = sts.rate_y - (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                    rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
                    //              sts.rate_y = (word)CO*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
                }
            }
            break;


        case 2: case 5:// flast motion
            //      FLAT_MOTION;
            //      TIMER_Y += sts.rate_y;
            if(sts.counter_y>=current_block->decelerate_after){
                current_block->state++;
                if(sts.rate_y>psettings->initial_rate)
                    sts.rate_y = psettings->initial_rate;

                //          max_rate = sts.rate_y;
            }
            break;

        case 3:// decelerate
            //      DOWN_SPEED_LAST;
            if(sts.speedLevel>current_block->final_speedLevel)
                sts.speedLevel--;
            if(sts.speedLevel>current_block->final_speedLevel){
                sts.rate_y = sts.rate_y + (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
            }else{
                sts.rate_y = current_block->final_rate;
            }
            break;

        case 4:
            //          DOWN_SPEED_FIRST;
            //          TIMER_Y += sts.rate_y;
            if(current_block->accelerate_until<=sts.counter_y){
                current_block->state++;
                sts.rate_y = current_block->nominal_rate;
                sts.speedLevel = current_block->speedLevel;
            }else{
                if(sts.speedLevel)
                    sts.speedLevel--;
                if(sts.speedLevel){
                    sts.rate_y = sts.rate_y + (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                    rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
                    if(sts.rate_y<current_block->initial_rate)
                        sts.rate_y = current_block->initial_rate;
                }
                else{
                    sts.rate_y = current_block->initial_rate;
                }
            }
            break;
        case 6:
            //          RISE_SPEED_LAST;
            if(sts.speedLevel < current_block->final_speedLevel)
                sts.speedLevel++;
            if(sts.speedLevel < current_block->final_speedLevel){
                sts.rate_y = sts.rate_y - (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
            }
            break;

        case 10:
            //          RISE_SPEED_FIRST;
            //          TIMER_Y += sts.rate_y;
            if(current_block->accelerate_until<=sts.counter_y){
                current_block->state++;
                sts.rate_y = current_block->nominal_rate;
            }
            if(sts.speedLevel < current_block->speedLevel)
                sts.speedLevel++;
            if(sts.speedLevel < current_block->speedLevel){
                sts.rate_y = sts.rate_y - (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
                //              sts.rate_y = psettings->initial_rate*(sqrtf(sts.speedLevel+1) - sqrtf(sts.speedLevel));
            }

            break;
        case 11:
            //  FLAT_MOTION;
            //          TIMER_Y += sts.rate_y;
            if(sts.counter_y>=current_block->decelerate_after){
                current_block->state++;
                //              sts.speedLevel--;
            }
            break;

        case 12:
            //  DOWN_SPEED_LAST;
            if(sts.speedLevel)
                sts.speedLevel--;
            if(sts.speedLevel){
                sts.rate_y = sts.rate_y + (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
            }
            break;



        case 24: // modal move, home
            //          RISE_SPEED_FIRST;
            //          TIMER_Y += sts.rate_y;
            if(sts.counter_y >= current_block->accelerate_until){
                current_block->state++;
                sts.rate_y = current_block->nominal_rate;
            }
            if(sts.speedLevel < current_block->speedLevel)
                sts.speedLevel++;
            if(sts.speedLevel < current_block->speedLevel){
                sts.rate_y = sts.rate_y - (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
            }

            break;

        case 25:
            //          FLAT_MOTION;
            //          TIMER_Y += sts.rate_y;
            if(sts.counter_y>=current_block->decelerate_after){
                current_block->state++;
            }
            break;

        case 26:
            //          DOWN_SPEED_LAST;
            if(sts.speedLevel)
                sts.speedLevel--;
            if(sts.speedLevel){
                sts.rate_y = sts.rate_y + (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
            }
            break;

        default:
            break;
        }
#endif
    NoOperation;
//    TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);
//    OS_LeaveInterrupt();
}


