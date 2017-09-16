/*
 * mscontrol.h
 *
 *  Created on: 2 авг. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_SCONTROL_H_
#define MSMOTOR_SCONTROL_H_

#include <stdint.h>

#define DOUBLE_n
#define CHAR_BIT    8

enum eDirections{
    forward = 1, backward,
};

struct sControl{
    uint8_t     axis;               // X_AXIS
//    uint32_t    linenumber;
    uint8_t     microsteps;         // 2
    uint8_t     initial_speedLevel; // 0
    uint8_t     speedLevel;         // 3
    uint32_t    steps;              //10
    uint32_t    accelerate_until;   // 3
    uint32_t    decelerate_after;   // 7
#ifdef DOUBLE
    float_t initial_rate;
#else
    uint32_t    initial_rate;       // 50132
#endif
#ifdef DOUBLE
    float_t nominal_rate;
#else
    uint32_t    nominal_rate;       // 5370
#endif
#ifdef DOUBLE
    float_t final_rate;
#else
    uint32_t    final_rate;         // 50132
#endif
    uint8_t     final_speedLevel;   // 0
    uint8_t     schem[3];           // 1, 2, 3
    enum eDirections        direction:CHAR_BIT; // uint8_t forward
    uint8_t     reseved1;
    uint16_t    reseved2;
};

/*
struct Stepper_state_t{
    uint32_t counter_y;     //  Текущая координата по оси Y.
    uint32_t point_y;       //  Расчётная точка по оси Y для следующего шага.
    uint32_t rate_y;        //  Темп движения по оси Y.
    uint8_t state;          //  состояние формирования скоростного режима.
    uint32_t speedLevel;    // Уровень скорости для разгона и торможения.
};
*/



#endif /* MSMOTOR_SCONTROL_H_ */
