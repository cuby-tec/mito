#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

/**
 * Project: 052KTF
 */

//#include "rtos.h"

#include <stdbool.h>

#define FALSE	0
#define TRUE	1

typedef signed char   int8;

//#ifndef uint16_t
//typedef unsigned short uint16_t;
//#endif

typedef unsigned short  uint16;
typedef short           int16;

typedef unsigned long uint32;
typedef long          int32;

//typedef unsigned long  int32_t;

typedef unsigned char uint8;
//typedef unsigned char	uint8_t;
//typedef unsigned char	uint8_t;

typedef unsigned char byte;
typedef unsigned int word;
typedef unsigned long dword;

typedef char* pString;

//typedef char* String;

//typedef unsigned long  uint32_t;

//typedef double  double_t;

//typedef int int32_t;

//---- GRBL
//#ifndef _MATH
//typedef float 	float_t;
//#endif
//typedef long int32_t;
//typedef  signed char int8_t;

//------- grbl
#ifdef IAR
#define NoOperation		asm("NOP\n")	// No operation 
#else
#define NoOperation     __asm  (" NOP\n")    // No operation
#endif

#ifndef TRUE
	#define TRUE      1
	#define FALSE     0
#endif

#ifndef OUTPUT
	#define OUTPUT    1
	#define INPUT     0
#endif

enum halfOctet{
  L,
  H
};

typedef enum {
  CU_OK = 1,
  CU_MEMORY,
  CU_ERROR,
  CU_TIMEOUT,
  CU_ERROR_WRONG_REPLAY
//  ,ERROR_TIMEOUT
}CU_CODE;

//typedef enum {
//  CHAR_CONFIG,
//  DATA_CONFIG
//}CU_MEMCONFIG;

//typedef struct {
//  byte* in;
//  byte* out;
//  short size_in;
//  short	size_out;
//}MEM_POOL;

//---- Константы -------
#define PI		3.14159265

#define NULL    0

#endif

