/*
 * sSegment.h
 *
 *  Created on: 24 авг. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_SSEGMENT_H_
#define MSMOTOR_SSEGMENT_H_

#include "sControl.h"
#include "msport.h"

/**
 * Заголовочная часть сегмента.
 */
struct sHead{
    uint8_t     axis_number;
    uint8_t     axis_mask;
    uint16_t    reserved;
    uint32_t    linenumber;
};

/**
 * Единица обмена данными.
 * Описатель отрезка траектории.
 */
struct sSegment
{
    struct sHead        head;

    struct sControl     axis[N_AXIS];
//    struct sControl     y_axis;
//    struct sControl     z_axis;
//    struct sControl     e_axis;
};

#endif /* MSMOTOR_SSEGMENT_H_ */
