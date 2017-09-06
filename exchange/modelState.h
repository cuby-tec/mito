/*
 * modelState.h
 *
 *  Created on: 4 сент. 2017 г.
 *      Author: walery
 */

#ifndef EXCHANGE_MODELSTATE_H_
#define EXCHANGE_MODELSTATE_H_

enum eModelState{
    eIdle=1,eWork,eWaitInstrument1,eWaitInstrument2,eWaitInstrument3,
    eWaitInstrument4,
    eExeption_ender1, eExeption_ender2, eExeption_ender3,
    eExeption_ender4, eExeption_ender5, eExeption_ender6
};


#endif /* EXCHANGE_MODELSTATE_H_ */
