/*
 * mscontrol.h
 *
 *  Created on: 2 авг. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_MSCONTROL_H_
#define MSMOTOR_MSCONTROL_H_


#define DOUBLE_n


enum eDirections{
    forward = 1, backward,
};

struct sControl{
    uint8_t     axis;
    uint32_t    linenumber;
    uint32_t    steps;
    uint8_t     microsteps;
    uint32_t    accelerate_until;
    uint32_t    decelerate_after;
#ifdef DOUBLE
    float_t initial_rate;
#else
    uint32_t    initial_rate;
#endif
    uint8_t     initial_speedLevel;
#ifdef DOUBLE
    float_t nominal_rate;
#else
    uint32_t    nominal_rate;
#endif
    uint8_t     speedLevel;
#ifdef DOUBLE
    float_t final_rate;
#else
    uint32_t    final_rate;
#endif
    uint8_t     final_speedLevel;
    uint8_t     schem[3];
    enum eDirections        direction; // uint8_t
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



#endif /* MSMOTOR_MSCONTROL_H_ */
