/*
 * HALmodele.h
 *
 *  Created on: 28 февр. 2018 г.
 *      Author: walery
 */

#ifndef DRIVERS_HALMODELE_H_
#define DRIVERS_HALMODELE_H_

#include "inc/typedefs.h"

#include "msmotor/msport.h"

//------------- defs

//-------------- vars


//--------------- function
 extern int32_t getEnders();

 extern int32_t getXminEnder();

// Назначить прерыввание при переходе из состояния Active->Passive (0->1).
extern void set_Xmin_IntType_rising();

// Назначить прерыввание при переходе из состояния Passive->Active (1->0).
extern void set_Xmin_IntType_falling();

//Обработка прерываний концевых выключателей. Port E
extern void PortEnderIntHandler();

#endif /* DRIVERS_HALMODELE_H_ */
