/*
 * status.h
 *
 *  Created on: 4 сент. 2017 г.
 *      Author: walery
 */

#ifndef EXCHANGE_STATUS_H_
#define EXCHANGE_STATUS_H_


#include "Status_t.h"

//---------- var

extern const struct Status_t    default_status;

//----------- function
extern struct Status_t* getStatus(void);

extern void init_Status();

#endif /* EXCHANGE_STATUS_H_ */
