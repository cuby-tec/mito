/*
 * eModelstate.h
 * Company: CUBY,Ltd
 * Project: freertos_demo
 *  Created on: 27 мая 2018 г.
 *      Author: walery
 */

#ifndef EXCHANGE_EMODELSTATE_H_
#define EXCHANGE_EMODELSTATE_H_


/*
 * - idle
- work
- wait_instrument1
- wait_instrument2
- exception
-- ender1
-- ender2
-- ender3
-- ender4
-- ender5
-- ender6
 */
enum eModelstate{
    ehIdle, ehIwork, ehWait_instrument1, ehWait_instrument2,
    ehException, ehEnder1, ehEnder2, ehEnder3, ehEnder4, ehEnder5, ehEnder6
};



#endif /* EXCHANGE_EMODELSTATE_H_ */
