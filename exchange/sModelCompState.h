/*
 * sModelCompState.h
 * Company: CUBY,Ltd
 * Project: freertos_demo
 *  Created on: 27 мая 2018 г.
 *      Author: walery
 */

#ifndef EXCHANGE_SMODELCOMPSTATE_H_
#define EXCHANGE_SMODELCOMPSTATE_H_

#include <stdint.h>
#include <stdbool.h>


struct sModelCompState{
    uint8_t modelState;
    uint8_t queueState;
    uint8_t reserved1;
    uint8_t reserved2;
};


#endif /* EXCHANGE_SMODELCOMPSTATE_H_ */
