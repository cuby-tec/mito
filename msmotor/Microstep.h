//
//
//  Generated by StarUML(tm) C++ Add-In
//
//  @ Project : 175-3d-printer
//  @ File Name : Microstep.h
//  @ Date : 01.01.2002
//  @ Author : 
//
//
/**
 * Table 1: Microstepping Resolution Truth Table
 * MS1  MS2 MS3     Microstep Resolution Excitation Mode
 * L    L   L       Full Step 2 Phase
 * H    L   L       Half Step 1-2 Phase
 * L    H   L       Quarter Step W1-2 Phase
 * H    H   L       Eighth Step 2W1-2 Phase
 * H    H   H       Sixteenth Step 4W1-2 Phase
 */

#if !defined(_MICROSTEP_H)
#define _MICROSTEP_H
#include <stdint.h>

// определяется количеством разрядов Расширителя порта.
#define MSG_LENGTH  3


#define MS1  (1)
#define MS2  (1<<1)
#define MS3  (1<<2)

#define Full_Step   (0)
#define Half_Step   (MS1)
#define Quarter_Step    (MS2)
#define Eighth_Step     (MS1|MS2)
#define Sixteenth_Step  (MS1|MS2|MS3)

#define EN_X    (1<<0)
#define EN_Y    (1<<1)
#define EN_Z    (1<<2)
#define EN_E0   (1<<3)
#define EN_E1   (1<<4)

struct Microstep {
	uint32_t E1:3;
	uint32_t EN:5;
	uint32_t Z:3;
	uint32_t E0:3;
	uint32_t em2:2;
	uint32_t X:3;
	uint32_t Y:3;
	uint32_t em1:2;
    uint32_t em0:8;
};

union uMicrostep {
    struct  Microstep microsteps;
    uint8_t data[4];
};


#endif  //_MICROSTEP_H
