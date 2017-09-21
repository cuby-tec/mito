/*
 * tSectorHandler.h
 *
 *  Created on: 29 авг. 2017 г.
 *      Author: walery
 */

#ifndef MSMOTOR_TSECTORHANDLER_H_
#define MSMOTOR_TSECTORHANDLER_H_


//------------ defs

//--------- signals
#define sg_segmentRecieved      (1)


//---------- vars
extern TaskHandle_t sectorHandling;

/**
 * Флаг перехода на новый сегмент:
 * - TRUE - переход завершён.
 * - FALSE - переход выполняется.
 */
extern bool sema_tail;


//--------- function
extern int32_t createTaskSectorHandler(void);

#endif /* MSMOTOR_TSECTORHANDLER_H_ */
