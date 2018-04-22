/*
 * sHotendControl.h
 *
 *  Created on: 22 апр. 2018 г.
 *      Author: walery
 */

#ifndef EXCHANGE_SHOTENDCONTROL_H_
#define EXCHANGE_SHOTENDCONTROL_H_



//------------- defs

/**
 * Включение/выключение нагревателя/вентилятора. false/true.
 */
struct sHotendSwitch {
    uint16_t cooler;
    uint16_t heater;
};

/**
 * Передача параметров управления инструментом Hotend.
 */
struct sHotendControl_t {
    float temperature;
    float kp;     // Коэффициент пропорциональной составляющей.
    float ki;     // Коэффициент интегральной составляющей.
    float kd;     // Коэффициент дифференциальной составляющей.
    struct sHotendSwitch _switch;    // Включение/выключение нагревателя/вентилятора. false/true.
};

//-------------- vars


//--------------- function



#endif /* EXCHANGE_SHOTENDCONTROL_H_ */
