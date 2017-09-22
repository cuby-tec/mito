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

//#include <grbl/settings.h>
//#include "grbl/planner.h"

#include "msmotor/block_state.h"
//#include "msmotor/mem_flash.h"

#include "msmotor/ms_model.h"
#include "msmotor/msport.h"
#include "msmotor/mempool.h"
#include "msmotor/ms_init.h"
#include "isrTimer.h"

#include "orderlyTask.h"

#include "Ktf_model.h"

#include "memf/tSectorHandler.h"
#include "memf/mSegmentQuee.h"

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

//static uint8_t rest = 0;

//static uint32_t speedRate;



uint32_t g_ui32Flags;

void (*ms_finBlock)(void);

//-------------- function


/**
 *  button control
 *  Получение нового сектора.
*/
void ms_nextSector()
{
static uint32_t be_portf1;
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
/*
static void ms_async_block(){
//    MS_ENABLE_OUT = MS_STOP; // Pause
//    MS_COMMAND_SEMA_SET;
    NoOperation;
}
*/




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

/*
        sync[1] |= X_FLAG;
        sync[1] |= Y_FLAG;
        sync[1] |= Z_FLAG;
        sync[1] |= E_FLAG;
*/
    sync[0] |= X_FLAG|Y_FLAG|Z_FLAG|E_FLAG;
    sync[1] |= X_FLAG|Y_FLAG|Z_FLAG|E_FLAG;

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

#ifdef V3
//    if(pusc)
//        HWREG(TIMER_BASE_X_AXIS + TIMER_O_RIS) |= TIMER_RIS_CAERIS;
#endif

}


volatile eTaskState state_task;
#define CONTINUE_

/**
 * Получение нового блока.
 * Отработка ломаной линии.
 * Контекст Прерывание
 */
void continueBlock(void)
{
    uint32_t timerValue;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool move_result;
//TODO Получить новый segment
    // проверить флаг sema_tail
    //      и вызвать функцию переопределения pblock_
    //      флаг sema_tail установить false
    if(sema_tail){
        move_result = move_pblock();
        if(move_result == TRUE){
            pMs_State->instrumrnt1 = eIns1_work; // work
            struct sSegment* segment = plan_get_current_block();
            pblockSegment(segment);
            sema_tail = FALSE;
        }
        else{
            pMs_State->instrumrnt1 = eIns1_stoped; // haven't segments.
        }
    }
//todo temporary blocked
//    if(pMs_State->instrumrnt1 == eIns1_stoped){
//        //todo аврийная ситуация: нет новых сегментов.
//        NoOperation;
//        return;
//    }

    if(axis_flags&X_FLAG){// таймер остановлен
        initStepper(X_AXIS);    // sts.counter = 0;
        sync[1] &= ~X_FLAG;

        sts.counter++;
        timerValue = sts.rate*multy;
        TimerLoadSet(TIMER_BASE_X_AXIS,TIMER_A,timerValue);
        timerValue >>=16;
        TimerPrescaleSet(TIMER_BASE_X_AXIS, TIMER_X, (timerValue&0x000000FF));
    }
    if(axis_flags&Y_FLAG){
        initStepper(Y_AXIS);
        sync[1] &= ~Y_FLAG;

        sts_y.counter++;
        timerValue = sts_y.rate*multy;
        TimerLoadSet(TIMER_BASE_Y_AXIS,TIMER_B,timerValue);
        timerValue >>=16;
        TimerPrescaleSet(TIMER_BASE_Y_AXIS, TIMER_Y, (timerValue&0x000000FF));
    }

    if(axis_flags&Z_FLAG){
        initStepper(Z_AXIS);
        sync[1] &= ~Z_FLAG;

        sts_z.counter++;
        timerValue = sts_z.rate*multy;
        TimerLoadSet(TIMER_BASE_Z_AXIS,TIMER_A,timerValue);
        timerValue >>=16;
        TimerPrescaleSet(TIMER_BASE_Z_AXIS, TIMER_Z, (timerValue&0x000000FF));
    }

    if(axis_flags&E_FLAG){
        initStepper(E_AXIS);
        sync[1] &= ~E_FLAG;

        sts_e.counter++;
        timerValue = sts_e.rate*multy;
        TimerLoadSet(TIMER_BASE_E_AXIS,TIMER_E,timerValue);
        timerValue >>=16;
        TimerPrescaleSet(TIMER_BASE_E_AXIS, TIMER_E, (timerValue&0x000000FF));

    }


    if(!(sync[1] & X_FLAG)
            && !(sync[1] & Y_FLAG)
            && !(sync[1] & Z_FLAG)
            && !(sync[1] & E_FLAG) )
    {
//        HWREG(TIMER_BASE_X_AXIS + TIMER_O_TAV) = 0;
//        HWREG(TIMER_BASE_Y_AXIS + TIMER_O_TAV) = 0;
//        HWREG(TIMER_BASE_Z_AXIS + TIMER_O_TAV) = 0;
//        HWREG(TIMER_BASE_E_AXIS + TIMER_O_TAV) = 0;

        if(sync[0] & X_FLAG){
//            TimerEnable(TIMER_BASE_X_AXIS, TIMER_X);
            HWREG(TIMER_BASE_X_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN;
            sync[1] |= X_FLAG;
        }
        if(sync[0] && Y_FLAG){
//            TimerEnable(TIMER_BASE_Y_AXIS, TIMER_Y);
            HWREG(TIMER_BASE_Y_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN;
            sync[1] |= Y_FLAG;
        }
        if(sync[0] & Z_FLAG){
//            TimerEnable(TIMER_BASE_Z_AXIS, TIMER_Z);
            HWREG(TIMER_BASE_Z_AXIS + TIMER_O_CTL) |= TIMER_CTL_TAEN;
            sync[1] |= Z_FLAG;
        }
        if(sync[0] & E_FLAG){
//            TimerEnable(TIMER_BASE_E_AXIS, TIMER_E);
            HWREG(TIMER_BASE_E_AXIS + TIMER_O_CTL) |= TIMER_CTL_TBEN;
            sync[1] |= E_FLAG;
        }
#ifndef DEBUG_NOTIFY
//        state_task = eTaskGetState(sectorHandling);
        //    xTaskNotifyFromISR(orderlyHandling,X_axis_int,eSetBits,&xHigherPriorityTaskWoken);
//        if(state_task == eSuspended){
//        ms_nextSector();    // Индикация
//            vTaskNotifyGiveFromISR(sectorHandling, &xHigherPriorityTaskWoken);
//        xSemaphoreGiveFromISR(memf_semaphor_handler,&xHigherPriorityTaskWoken);
//        xTaskNotifyFromISR(sectorHandling,SECTOR_TO_RELEAS,eSetBits,&xHigherPriorityTaskWoken);
        sema_tail = TRUE;
        xTaskNotifyGive(sectorHandling);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        }
#endif
    }
}

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

    if(sync[0] & X_FLAG & sync[1]){
        sync[1] &= ~X_FLAG;
        TimerDisable(TIMER_BASE_X_AXIS, TIMER_X);
    }

    if(sync[0] & Y_FLAG & sync[1]){
        sync[1] &=~ Y_FLAG;
        TimerDisable(TIMER_BASE_Y_AXIS, TIMER_Y);
    }

    if(sync[0] & Z_FLAG & sync[1]){
        sync[1] &=~ Z_FLAG;
        TimerDisable(TIMER_BASE_Z_AXIS, TIMER_Z);
    }

    if(sync[0] & E_FLAG & sync[1]){
        sync[1] &=~ E_FLAG;
        TimerDisable(TIMER_BASE_E_AXIS, TIMER_E);
    }


    if(!(sync[1] & X_FLAG)
            && !(sync[1] & Y_FLAG)
            && !(sync[1] & Z_FLAG)
            && !(sync[1] & E_FLAG))
    {
//        flag = 1;
    xTaskNotifyFromISR(orderlyHandling,X_axis_int_fin,eSetBits,NULL);
    }
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

