/*
 * ms_settings.h
 *
 *  Created on: 28 февр. 2018 г.
 *      Author: walery
 */

#ifndef KALIBROVKA_MS_SETTINGS_H_
#define KALIBROVKA_MS_SETTINGS_H_



//------------- defs
// Кооличество шагов двигателя за оборот для полного шага.
#define MOTOR_STEPS_X   200

//-------------- vars
/**
 * Максимальный размер по оси X.
 * Размер указывается в миллимертах.
 */
extern float ms_Xmax;

/**
 * Длина шага в миллиметрах.
 * Размер задаётся для полного шага двигателя.
 */
extern float ms_Xstep;

//--------------- function



#endif /* KALIBROVKA_MS_SETTINGS_H_ */
