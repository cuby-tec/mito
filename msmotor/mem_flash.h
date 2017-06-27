/*
 * mem_flash.h
 * Company: CUBY
 *  Created on: 23.11.2013
 *      Author: walery
 *
 *       Хранение данных в постоянной памяти.
 *
 */

#ifndef MEM_FLASH_H_
#define MEM_FLASH_H_
#include "inc/typedefs.h"
//#include <settings.h>
#include "grbl/settings_t.h"

//---------- Definition -------


#define SEGMENT_N			(0x2500)	// In MSP430F1612 size 256 byte
#define SEGMENT_N_SIZE		256

#define SEGMENT_STORE		SEGMENT_N
#define SEGMENT_STORE_SIZE	SEGMENT_N_SIZE


//----------- Vars ----------
//#pragma object_attribute = __no_init
//__no_init  volatile settings_t store_setting @SEGMENT_STORE;
//const settings_t store_setting @SEGMENT_STORE;
//extern  const  int x;
extern const settings_t settings;

extern settings_t* psettings;

//---------- Function --------

void mf_checkState();

#endif /* MEM_FLASH_H_ */
