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
    uint32_t    linenumber;
    uint8_t     axis_number;
    uint32_t    axis_mask;
};

/**
 * Единица обмена данными.
 * Описатель отрезка траектроии.
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
