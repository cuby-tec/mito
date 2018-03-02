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

void kalibrovka (void)
{

    struct sSegment* psc;
    uint32_t ulNotificationValue;
    BaseType_t ret;
    psc = plan_get_current_block();

//    IntEnable (INT_GPIOE);

    // Проверка состояния концевого выключателя оси Xmin.

    if(getXminEnder() == ENDER_ACTIVE){
        // Инструмент находится в рабочей области.
        NoOperation; //TODO XMIN-Ender
        //- определить фронт/спад прерывания концевика: Acive to Passive;
        set_Xmin_IntType_rising();
        //- Определить направление движения и разрешённые оси;
        if(psc){
            buildSegment_MoveToXmin(psc);

            start_xkalibrovka();
            //TODO wait end move
            //            if(xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY)==pdTRUE){
ka1:            ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(2000));//portMAX_DELAY
            if(ret == pdTRUE){
                // ender Xmin acheved
                if(ENDER_XMIN_HANDLE & ulNotificationValue){

                    NoOperation;
                }
            }else{
                // каретка не достигла концевого выклюяателя.
                NoOperation;
                start_xkalibrovka();
                goto ka1;
            }
        }
        else
            while(1){
                NoOperation; // Останов по ошибке: недостаточно памяти HEAP
            }

    }
    // Инструмент находится за пределами рабочей области.
    NoOperation; //TODO XMIN-Ender-revers

    // Калибровка по оси X-min


}
