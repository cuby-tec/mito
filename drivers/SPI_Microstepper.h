/*
 * SPI_Microstepper.h
 *
 *  Created on: 8 дек. 2017 г.
 *      Author: walery
 */

#ifndef DRIVERS_SPI_MICROSTEPPER_H_
#define DRIVERS_SPI_MICROSTEPPER_H_


//#include "inc/typedefs.h"
#include <stdint.h>
//------------- defs

//-------------- vars


//--------------- function
// Configure SPI2 port.
void SPI_init(void);

void SPI_Send(uint8_t* data,uint8_t count);

void SPI_Read(uint8_t* data,uint8_t count);


#endif /* DRIVERS_SPI_MICROSTEPPER_H_ */
