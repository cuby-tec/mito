/*
 * ms_kalibrovka.c
 *
 *  Created on: 28 февр. 2018 г.
 *      Author: walery
 */

//--------------


#include "ms_kalibrovka.h"

#include <stdint.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <limits.h>

#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

#include "drivers/HALmodele.h"
#include "memf/cnc_sector.h"
#include "msmotor/sSegment.h"
#include "memf/mSegmentQuee.h"
#include "msmotor/ms_model.h"
#include "msmotor/ms_init.h"
#include "switch_task.h"

//------------- defs

//-------------- vars


//-------------- function


byte checkEnder (void){
    byte result = 0;

    result = getEnders();

    return result;
}

void moveToXmin(void){

}

void reversToXmin(void){

}

void errorKalibrovka(void){

}

uint32_t interrupt_counter = 0;

// Ошибка при калибровке.
static void kalibrovka_exception(uint32_t code)
{
//    while(1){
        NoOperation;
//    }
}


enum kalibrovka_state {
    kalibrovka_xmin_decrease = 1,   // Движение в сторону оси Xmin с уменьшением координаты.
    kalibrovka_xmin_increase,   // Движение по оси Xmin с увеличением координаты
    kalibrovka_ymax_decrease,   // Движение по оси Y в сторону уменьшения координаты.
    kalibrovka_ymax_increase    // Движение поо оси Y в сторону увеличения координаты.
};

static bool kalibrovka_move(enum kalibrovka_state kState)
{
    bool result = false;
    struct sSegment* psc;
    static uint32_t ulNotificationValue;

    NoOperation;
    psc = plan_get_current_block();

    switch(kState)
    {

    case kalibrovka_xmin_decrease:
        result = false;
        //- назначить фронт/спад прерывания концевика: Acive to Passive;
//        set_Xmin_IntType_rising();
        set_Xmin_IntType_falling();
        //- Назначить направление движения и разрешённые оси;
        buildSegment_MoveToXmin(psc);
        start_xkalibrovka();
        ka1:
        // wait end move
        if(xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(500))==pdTRUE)//portMAX_DELAY
        {
            // ender Xmin achieved
            if(ENDER_XMIN_HANDLE & ulNotificationValue){
                NoOperation;
                result = true;
            }
        }else{
            // каретка не достигла концевого выклюяателя.
            NoOperation;
            start_xkalibrovka();
            goto ka1;
        }
        break;

    case kalibrovka_xmin_increase:
        result = false;
//        set_Xmin_IntType_falling();
        set_Xmin_IntType_rising();
        buildSegment_MoveToXmax(psc);
        start_xkalibrovka();
        // wait end move
        if(xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(2000))==pdTRUE)//portMAX_DELAY
        {
            // ender Xmin achieved
            if(ENDER_XMIN_HANDLE & ulNotificationValue){
                NoOperation;
                result = true;
            }
        }else{
            // каретка не достигла концевого выклюяателя.
            kalibrovka_exception(123);
        }

        break;

    }// end switch(kState)

    return result;
}



void kalibrovka (void)
{
    enum kalibrovka_state state;

//    IntEnable (INT_GPIOE);
    interrupt_counter = 0;
    // Проверка состояния концевого выключателя оси Xmin.

    if(getXminEnder() == ENDER_ACTIVE){
        // Инструмент находится в рабочей области.
        NoOperation; //TODO XMIN-Ender

        state = kalibrovka_xmin_decrease;
//        Движение в сторону оси Xmin с уменьшением координаты.
        if(kalibrovka_move(state))
        {
            NoOperation;
            state = kalibrovka_xmin_increase;
            vTaskDelay(pdMS_TO_TICKS(200));    // минимальное значение 1000

        }else{
            NoOperation;
            kalibrovka_exception(155);
        }


        if(kalibrovka_move(state))
        {
            state = kalibrovka_ymax_increase;
        }else{
            kalibrovka_exception(163);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        NoOperation;
        disableMotors();
    }
    // Инструмент находится за пределами рабочей области.
    NoOperation; //TODO XMIN-Ender-revers

    // Калибровка по оси X-min


}
