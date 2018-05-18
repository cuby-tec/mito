/*
 * usbmodule.h
 *
 *  Created on: 28 июл. 2017 г.
 *      Author: walery
 */

#ifndef DRIVERS_USBMODULE_H_
#define DRIVERS_USBMODULE_H_


#include <stdbool.h>
#include "orderlyTask.h"



#define  sendStatus_p_NO

uint8_t usb_init(void);

uint32_t sendStatus();



#if (sendStatus_p == 3 || sendStatus_p == 4)
extern bool commandFlag;
#endif



#endif /* DRIVERS_USBMODULE_H_ */
