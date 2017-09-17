/*
 * ms_state.h
 *
 *  Created on: 17 сент. 2017 г.
 *      Author: walery
 *      Описание состояния устройства.
 */

#ifndef MSMOTOR_MS_STATE_H_
#define MSMOTOR_MS_STATE_H_

#include <stdint.h>

//------------- defs

enum eInstrument1{
    eIns1_work=1,
    eIns1_stoped    // нет новых сегментов в буфере Сегментов.
    ,
};


struct sMs_State{
    enum eInstrument1 instrumrnt1;
};


//-------------- vars


//--------------- function



#endif /* MSMOTOR_MS_STATE_H_ */
