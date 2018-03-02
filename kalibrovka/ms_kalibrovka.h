/*
 * ms_kalibrovka.h
 *
 *  Created on: 28 февр. 2018 г.
 *      Author: walery
 */

#ifndef KALIBROVKA_MS_KALIBROVKA_H_
#define KALIBROVKA_MS_KALIBROVKA_H_


#include "inc/typedefs.h"
//------------- defs

//-------------- vars


//--------------- function
/** выпонение действий Калибровки в контексте Task
 * (Должна вызываться из компонента типа Task).
 */
extern void kalibrovka (void);

/**
 * Проверка состояния концевых выключателей.
 */
extern uint8 checkEnder (void);

/**
 * Происходит движение Инструмента в сторону
 * оси Xmin до срабатывания концевика X-min-ender.
 * Уменьшение значения координаты X.
 */
extern void moveToXmin(void);

/**
 * Движение в сторону увеличения значения кооординаты X.
 * Происходит движение Инструмента в сторону оси Xmax
 * до срабатывания концевика X-min-ender.
 */
extern void reversToXmin(void);

/**
 * В процессе калибровки оси возникла ошибка,
 * состоящая в том, что на максимальной дистанции не сработал
 * концевой выключатель.
 * Причины:
 * - неисправность двигателя
 * - Неисправность шкива
 * - неисправность ремня
 * - неисправность каретки (заклинивание).
 */
extern void errorKalibrovka(void);


#endif /* KALIBROVKA_MS_KALIBROVKA_H_ */
