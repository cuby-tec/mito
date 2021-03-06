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
    kl_xmin_fall,    // Xmin
    kl_xmax_rise,    // Xmax
    kl_xmax_fall,   // Xmax
    kl_ymax_fall,   // Ymax
    kl_ymax_rise,   // Ymax
    kl_ymin_fall,   // Ymin
    kl_ymin_rise,   // Ymin
    kl_zmax_fall,   // Zmax
    kl_zmax_rise,   // Zmax
    kl_zmin_fall,   // Zmin
    kl_zmin_rise    // Zmin
};


#define ALL_ENDERS  (ENDER_Z_MIN|ENDER_Z_MAX \
                         |ENDER_Y_MIN|ENDER_Y_MAX \
                         |ENDER_X_MIN|ENDER_X_MAX)

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
