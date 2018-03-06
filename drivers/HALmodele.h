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

enum ender_edge{
    kl_xminrise=1,  // Xmin
    kl_xminfall,    // Xmin
    kl_xmaxrise,    // Xmax
    kl_xmax_fall    // Xmax
};

//-------------- vars


//--------------- function
 extern uint32_t getEnders();

 extern uint32_t getXminEnder();

 extern bool getXmaxEnder();

// Назначить прерыввание при переходе из состояния Active->Passive (0->1).
extern void set_Xmin_IntType_rising();

// Назначить прерыввание при переходе из состояния Passive->Active (1->0).
extern void set_Xmin_IntType_falling();

//Обработка прерываний концевых выключателей. Port E
extern void PortEnderIntHandler();

// назначение прерывания
extern void set_EnderEdge(enum ender_edge edge);

extern void set_DisableIntEnders(void);

#endif /* DRIVERS_HALMODELE_H_ */
