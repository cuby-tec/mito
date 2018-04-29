/*
 * sProfile.h
 *
 *  Created on: 29 апр. 2018 г.
 *      Author: walery
 */

#ifndef EXCHANGE_SPROFILE_H_
#define EXCHANGE_SPROFILE_H_

#include <stdint.h>
#include <stdbool.h>

//------------- defs
struct sProfile{
    float X_STEPS_PER_MM;
    float Y_STEPS_PER_MM;
    float Z_STEPS_PER_MM; // 250,00,
    float X_MAX_RATE; // 500,00,
    float Y_MAX_RATE; // 500,00,
    float Z_MAX_RATE; // 500,00,
    float X_ACCELERATION; // 36000,00,
    float Y_ACCELERATION; // 36000,00,
    float Z_ACCELERATION; // 36000,00,
    float X_MAX_TRAVEL; // 200,00,
    float Y_MAX_TRAVEL; // 200,00,
    float Z_MAX_TRAVEL; // 200,00,
    float STEP_PULSE_MICROSECONDS; // 10,
    float STEPPING_INVERT_MASK; // 0,
    float DIRECTION_INVERT_MASK; // 0,
    float STEPPER_IDLE_LOCK_TIME; // 25,
    float STATUS_REPORT_MASK; // 3,
    float JUNCTION_DEVIATION; // 0,01,
    float ARC_TOLERANCE; // 0,0020,
    float FLAGS; // 0,
    uint32_t HOMING_DIR_MASK; // ,
    float HOMING_FEED_RATE; // 25,00,
    float HOMING_SEEK_RATE; // 500,00,
    uint32_t HOMING_DEBOUNCE_DELAY; // 250,
    float HOMING_PULLOFF; // 1,00
    float DERIVATIVE; // 0.12,
    float INTEGRAL; // 0.18,
    float PROPTIONAL; // 0.7,
    float TEMPERATURE; // 50
};//-------------- vars


//--------------- function



#endif /* EXCHANGE_SPROFILE_H_ */
