/*
 * ms_model.c
 *
 *  Created on: 21 июн. 2017 г.
 *      Author: walery
 */



//--------------
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "inc/sysDrivers.h"



#ifdef INC_FREERTOS_H
#define malloc(size) pvPortMalloc(size)
#define free(ptr) pvPortFree(ptr)
#endif


//#include <sysbiosHeader.h>
#include "drivers/pinout.h"

#include "inc/typedefs.h"

#include <grbl/settings.h>
//#include "grbl/planner.h"

#include "msmotor/block_state.h"
#include "msmotor/mem_flash.h"

#include "msmotor/ms_model.h"
#include "msmotor/msport.h"
#include "msmotor/mempool.h"
#include "msmotor/ms_init.h"


#include "orderlyTask.h"

#include "Ktf_model.h"



//------------- defs

#define PE0     GPIO_PIN_0
#define PE1     GPIO_PIN_1
#define PE2     GPIO_PIN_2
#define PE3     GPIO_PIN_3
#define PE4     GPIO_PIN_4


#define LED_PIN2_no
#define LED_PIN3_no
#define LED_PIN1

#define Y_STEP_BIT          7  // Port 5 pin 7
#define Y_DIRECTION_BIT     6  // Port 5 Pin 6
#define STEP_MASK           (1<<Y_STEP_BIT) // All step bits
#define DIRECTION_MASK      (1<<Y_DIRECTION_BIT) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

// #define CE HWREGBITW((GPIO_PORTA_BASE +(GPIO_O_DATA+(GPIO_PIN_0))),0)   //PA0
//Block end
#define  BE  HWREGBITW((GPIO_PORTF_BASE +(GPIO_O_DATA)),1)



//-------------- vars

//static float start_speed = 0.0066;
byte direction = false;

//static void (*ms_finBlock)(void);

static uint32_t multy = 1;

static uint8_t rest = 0;

//static uint32_t speedRate;

static uint32_t be_portf1;


uint32_t g_ui32Flags;

void (*ms_finBlock)(void);

//-------------- function

#ifdef commit_13
/**
 * Обновление значений счётчика оси X
 */
void axisX_rateHandler()
{
    // Обработка следующегошага.
        axis_flags &= ~Y_FLAG;
#ifdef LED_PIN3
        HWREGBITW(&g_ui32Flags,3) ^= 1;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, g_ui32Flags);
//            0x400253fc
#endif
        if(sts.counter > sts.point){
             (*ms_finBlock)();
#ifdef LED_PIN2
             GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
#endif
//               return;
        }//else

        switch(pblock->schem[sts.state]){
        case 1:
            //      RISE_SPEED_FIRST;
            //      TIMER_Y += sts.rate_y;
            rest = 0;
            if(current_block->accelerate_until<=sts.counter){
                sts.state++;
                speedRate = sts.rate;
                sts.rate = current_block->nominal_rate;
                sts.speedLevel = current_block->speedLevel;
            }else{
                if(sts.speedLevel < current_block->speedLevel){
                    sts.speedLevel++;
                    if(sts.speedLevel == 1){
                        sts.rate *= 0.4056;
                    }
                    else{
#ifdef DOUBLE
                        sts.rate = current_block->initial_rate*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
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
            if(sts.counter>=current_block->decelerate_after){
                sts.state++;
//                if(sts.rate_y>psettings->initial_rate)
//                    sts.rate_y = psettings->initial_rate;
//                sts.rate_y = current_block->final_rate;
                sts.rate = speedRate;
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
            if(sts.speedLevel>current_block->final_speedLevel){
#ifdef DOUBLE
                sts.rate = current_block->initial_rate*(sqrtf(sts.speedLevel+1)-sqrtf(sts.speedLevel));
#else
//                sts.rate_y = sts.rate_y + (((2 * (long)sts.rate_y) + rest)/(4 * sts.speedLevel + 1));
                sts.rate = sts.rate + (((2 * (long)sts.rate))/(4 * sts.speedLevel + 1 + rest));
//                rest = ((2 * (long)sts.rate_y)+rest)%(4 * sts.speedLevel + 1);
#endif
            }else{
                sts.rate = current_block->final_rate;
            }
            break;
        }


//           printReport();
#ifdef LED_PIN2
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
#endif
   // end handler Y axis.

}
#endif //commit#13


/**
 *  button control
 *  Синхронная работа считывателя блоков.
*/
void ms_nextBlock()
{
//    uint32_t be = GPIO_PORTF_BASE +  0x3fc;
//    uint32_t* abe = GPIO_PORTF_BASE +  0x3fc;
//  MS_ENABLE_OUT = MS_STOP; // Pause
//    if(ktf_model.block_counter == block_target){
//        NoOperation;
//    }

//    MS_COMMAND_SEMA_IE; // Ожидает достижения координаты X
//    STOP_Y;
#ifdef LED_PIN1
    HWREGBITB(&be_portf1,1) = ~HWREGBITB(&be_portf1,1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, be_portf1);
#endif
//    be = *abe;
//    be = BE;
//    BE = ~be;
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




#define V3
//Запуск таймера оси.
void start_t1(uint8_t pusc)
{
#ifndef V3
    uint32_t timerValue,timerValueMatch;
    flag = 0;
#endif


#ifdef V3_

    TimerLoadSet(TIMER_BASE_X_AXIS,TIMER_A,timerLoad);
    TimerMatchSet(TIMER_BASE_X_AXIS, TIMER_A, timermatch);
    TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_A, 0);
    TimerPrescaleMatchSet(TIMER_BASE_X_AXIS, TIMER_A, 0);
#endif

#ifndef V3
    timerValue = sts.rate*multy;
    timerValueMatch = timerValue/3;

    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = (uint16_t)(timerValue&0x0000FFFF);
    timerValue >>=16;
    TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_A, (uint16_t)(timerValue&0x000000FF));

    TimerMatchSet(TIMER_BASE_X_AXIS, TIMER_A, (timerValueMatch&0x0000FFFF));
    timerValueMatch >>=16;
    TimerPrescaleMatchSet(TIMER_BASE_X_AXIS, TIMER_A, (uint16_t)(timerValueMatch&0x000000FF));

    dump[0] = TimerLoadGet(TIMER_BASE_X_AXIS, TIMER_A);
#endif

    initStepper(N_AXIS);  // next block
    if(pusc == 0){
        Timer1IntHandler();
        TimerYIntHandler();
        TimerZIntHandler();
        TimerEIntHandler();
    }


//    if(sync[0] & X_FLAG){
//        TimerEnable(TIMER_BASE_X_AXIS, TIMER_X);
    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN;
//    }

//    if(sync[0] & Y_FLAG){
//        TimerEnable(TIMER_BASE_Y_AXIS, TIMER_Y);
    HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN;
//    }

//    if(sync[0] & Z_FLAG){
//        TimerEnable(TIMER_BASE_Z_AXIS, TIMER_Z);
    HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN;
//    }

//    if(sync[0] & E_FLAG){
//        TimerEnable(TIMER_BASE_E_AXIS, TIMER_E);
    HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN;
//    }
        sync[1] |= X_FLAG;
        sync[1] |= Y_FLAG;
        sync[1] |= Z_FLAG;
        sync[1] |= E_FLAG;


#ifdef V3
//    if(pusc)
//        HWREG(TIMER_BASE_X_AXIS + TIMER_O_RIS) |= TIMER_RIS_CAERIS;
#endif

}



#define CONTINUE_
/**
 * Получение нового блока.
 * Отработка ломаной линии.
 */
/*
void continueBlock()
{
    uint32_t timerValue;

//TODO Получить новый блок
    initStepper(X_AXIS);

    sts.counter++;
    timerValue = sts.rate*multy;
    TimerLoadSet(TIMER_BASE_X_AXIS,TIMER_A,timerValue);
    timerValue >>=16;
    TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_A, (timerValue&0x000000FF));
    xTaskNotifyFromISR(orderlyHandling,X_axis_int,eSetBits,NULL);

}
*/
/*

eTaskState getSate()
{
    eTaskState result;
    result = eTaskGetState(orderlyHandling);
    return result;
}

void stub(uint32_t tmp)
{
    uint32_t s = tmp;
}
*/

void exitBlock(void)
{
    eTaskState et;
    uint32_t cnt;
//     printf("\n======== Exit block. =========\n\n");
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_IMR) &= ~TIMER_TAMR_TAPWMIE; //TIMER_TAMR_TAPWMIE
//    IntEnable(INT_TIMER1A);
//    IntDisable(INT_TIMER1A);
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_IMR) &= ~TIMER_IMR_CAEIM;
//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMR) &= ~TIMER_TAMR_TAPWMIE;
//    TimerIntDisable(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);   // work
#ifndef CONTINUE
    TimerDisable(TIMER_BASE_X_AXIS, TIMER_A);

    xTaskNotifyFromISR(orderlyHandling,X_axis_int_fin,eSetBits,NULL);
/*
    et = getSate();
    if(et == eReady)
        cnt = 1;
    else if(et == eBlocked)
        cnt = 2;
    else if(et == eSuspended)
        cnt =3;
    else if(et == eDeleted)
        cnt = 4;
    else if(cnt == eRunning)
        cnt = 5;
//    stub(cnt);
    HWREGBITB(&g_ui32Flags,cnt) = 1;
*/
//    flag = 1; X_axis_int_fin

#else
    uint32_t timerValue;

    initStepper();

    sts.counter++;
    timerValue = sts.rate*multy;
    TimerLoadSet(TIMER_BASE_X_AXIS,TIMER_A,timerValue);
    timerValue >>=16;
    TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_A, (timerValue&0x000000FF));


#endif
/*
 *     //
    // Configure the GPIO Pin Mux for PB4
    // for T1CCP0
    //
    MAP_GPIOPinConfigure(GPIO_PB4_T1CCP0);
    MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4);
 */
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
//    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x0);

//    HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) &= ~TIMER_CTL_TAEN; //
//    TimerDisable(TIMER_BASE_X_AXIS, TIMER_A);
}




//------------- interrupt

// Keep track of remainder from new_step-delay calculation to increase accuracy
#define DEBUG_INT_no
#ifdef commit_13
/**
 * Обработчик прерываний оси X
 */
void axisX_intrrupt_handler(void){//    uint32_t cOne = 1, cTwo;

    static uint32_t timerValue_Y;
//    uint32_t timerValueMatch;

    TimerIntClear(TIMER_BASE_X_AXIS, TIMER_CAPA_EVENT);
//    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);

    // Toggle the flag for the second timer.
//    HWREGBITW(&g_ui32Flags, 1) ^= 1;
#ifdef LED_PIN2
    // Use the flags to Toggle the LED for this timer
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
#endif
//if(sts.counter_y == 0)
//    sts.counter_y = 0;
//else if(sts.counter_y == 1)
//    sts.counter_y = 1;
//else if(sts.counter_y == 2)
//    sts.counter_y = 2;
//else if(sts.counter_y == 3)
//    sts.counter_y = 3;
//else if(sts.counter_y == 4)
//    sts.counter_y = 4;
//else if(sts.counter_y == 5)
//    sts.counter_y = 5;

    sts.counter++;

           timerValue_Y = sts.rate*multy;
//           timerValueMatch = timerValue - 128;// timerValue/8;
//           dump[ sts.counter_y%60] = sts.rate_y;

           TimerLoadSet(TIMER_BASE_X_AXIS,TIMER_A,timerValue_Y);
//           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = sts.rate_y*multy;
//           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAILR) = (uint16_t)(timerValue&0x0000FFFF);
           timerValue_Y >>=16;
           TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_A, (uint16_t)(timerValue_Y&0x000000FF));
//           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAPR) = (uint16_t)(uint16_t)(timerValue&0x000000FF);

/*
//           TimerMatchSet(TIMER_BASE_X_AXIS, TIMER_A, (timerValueMatch&0x0000FFFF));
           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAMATCHR) = (uint16_t)(timerValueMatch&0x0000FFFF);
           timerValueMatch >>=16;
//           TimerPrescaleMatchSet(TIMER_BASE_X_AXIS, TIMER_A, (uint16_t)(timerValueMatch&0x000000FF));
           HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAPMR) = (uint16_t)(timerValueMatch&0x000000FF);
*/

//           axis_flags |= Y_FLAG;
           axisX_rateHandler();
//           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
           return;
}
#endif

