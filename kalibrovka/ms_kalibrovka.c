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
#define NOTIFY_WAIT_MS  400
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

//uint32_t interrupt_counter = 0;

// Ошибка при калибровке.
static void kalibrovka_exception(uint32_t code)
{
    disableMotors();
    while(1){
        NoOperation;
    }
}


enum kalibrovka_state {
    kalibrovka_xmin_decrease = 1,   // Движение в сторону оси Xmin с уменьшением координаты.
    kalibrovka_xmin_increase,   // Движение по оси Xmin с увеличением координаты
    kalibrovka_xmax_increase,   // Калибровка датчика Xmax.
    kalibrovka_xmax_decrease,
    kalibrovka_ymax_decrease,   // Движение по оси Y в сторону уменьшения координаты.
    kalibrovka_ymax_increase    // Движение поо оси Y в сторону увеличения координаты.
};

#define BitsToClearOnEntry (ENDER_XMIN_HANDLE | ENDER_XMAX_HANDLE)

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
//        set_Xmin_IntType_falling();
        set_EnderEdge(kl_xminfall);
        //- Назначить направление движения и разрешённые оси;
        buildSegment_MoveToXmin(psc);
        ulNotificationValue = 0;
 ka1:       start_xkalibrovka(X_AXIS);
 //        ka1:
 // wait end move
 kl11:      if(xTaskNotifyWait(ENDER_XMAX_HANDLE, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(NOTIFY_WAIT_MS))==pdTRUE)//portMAX_DELAY
 {
     // ender Xmin achieved
     if(ENDER_XMIN_HANDLE & ulNotificationValue){
         NoOperation;
         result = true;
     }else{
         NoOperation;// Сигнал от другого концевого выключателя.
         goto kl11;
     }
 }else{
     // каретка не достигла концевого выклюяателя.
     NoOperation;
     //            start_xkalibrovka(X_AXIS);
     if(getEnders() & ENDER_X_MIN){
//         result = true;1
         goto ka1;
     }else{
         result = true;
//         goto ka1;
     }
 }
 break;

    case kalibrovka_xmin_increase:
        result = false;
//        set_Xmin_IntType_falling();
//        set_Xmin_IntType_rising();
        set_EnderEdge(kl_xminrise);
        //        buildSegment_MoveToXmax(psc);
        kl_buildSement(psc, kl_Xforward);
        start_xkalibrovka(X_AXIS);
        // wait end move
        kl_kxi:    if(xTaskNotifyWait(ENDER_XMAX_HANDLE, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(NOTIFY_WAIT_MS))==pdTRUE)//portMAX_DELAY
        {
            // ender Xmin achieved
            if(ENDER_XMIN_HANDLE & ulNotificationValue){
                NoOperation;
                result = true;
            }else{
                goto kl_kxi;
            }
        }else{
            // каретка не достигла концевого выклюяателя.
            kalibrovka_exception(133);
        }
        break;

    case kalibrovka_xmax_increase:
        result = false;
        ulNotificationValue = 0;
        set_EnderEdge(kl_xmax_fall);
        kl_buildSement(psc, kl_Xforward);
kl2:        start_xkalibrovka(X_AXIS);
kl2_a:        if(xTaskNotifyWait(ENDER_XMIN_HANDLE, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(NOTIFY_WAIT_MS))==pdTRUE)//portMAX_DELAY
        {
            // ender Xmax achieved
            if(ENDER_XMAX_HANDLE & ulNotificationValue){
                NoOperation;
                result = true;
            }else{
                goto kl2_a;
            }
        }else{
            // каретка не достигла концевого выклюяателя.
            goto kl2;
        }
        break;

    case kalibrovka_xmax_decrease:
        result = false;
        set_EnderEdge(kl_xmaxrise);
        kl_buildSement(psc, kl_Xbackward);
        start_xkalibrovka(X_AXIS);
        kl3_kxd:    if(xTaskNotifyWait(ENDER_XMIN_HANDLE, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(NOTIFY_WAIT_MS))==pdTRUE)//portMAX_DELAY
        {
            // ender Xmin achieved
            if(ENDER_XMAX_HANDLE & ulNotificationValue){
                NoOperation;
                result = true;
            }else{
                goto kl3_kxd;
            }
        }else{
            // каретка не достигла концевого выклюяателя.
            kalibrovka_exception(170);
        }

        break;

    }// end switch(kState)

    return result;
}


    enum kalibrovka_state state;

void kalibrovka (void)
{

    //    IntEnable (INT_GPIOE);
//    interrupt_counter = 0;
    // Проверка состояния концевого выключателя оси Xmin.
    set_DisableIntEnders();
    // Калибровка по оси X-min
    if(getXminEnder() == ENDER_ACTIVE){
        // Инструмент находится в рабочей области.
        NoOperation; //TODO XMIN-Ender

        state = kalibrovka_xmin_decrease;
        //        Движение в сторону оси Xmin с уменьшением координаты.
        if(kalibrovka_move(state))
        {
            NoOperation;
            state = kalibrovka_xmin_increase;
            vTaskDelay(pdMS_TO_TICKS(200));

        }else{
            NoOperation;
            kalibrovka_exception(204);
        }
    }
    // Инструмент находится за пределами рабочей области.
    NoOperation; // kalibrovka_xmin_increase
    if(kalibrovka_move(state))
    {
        state = kalibrovka_xmax_increase;
    }else{
        kalibrovka_exception(213);
    }

    // Калибровка Xmax increase
    if(kalibrovka_move(state))
    {
        vTaskDelay(pdMS_TO_TICKS(200));
        state = kalibrovka_xmax_decrease;
    }else{
        kalibrovka_exception(221);
    }

    // Калибровка Xmax decrease.
    if(kalibrovka_move(state))
    {
        NoOperation;
    }else{
        kalibrovka_exception(228);
    }

    // Калибровка по оси Ymax




    vTaskDelay(pdMS_TO_TICKS(1000));
    NoOperation;
    disableMotors();



}
