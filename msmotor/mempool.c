/*
 * mempool.c
 *
 *  Created on: 26.09.2010
 *      Author: Администратор
 */

#include "mempool.h"



//-------- Vars
#ifdef KTF7
OS_MEMF cmdPool;
#else
// stub
uint16_t cmdPool[10];
#endif
union{

	char ff_filebuffer[14][12];
	block_state block_buffer[BLOCK_BUFFER_SIZE];

};




//-------------- Function


void createCommandPool(){
#ifdef KTF7
	OS_MEMF_Create(&cmdPool,block_buffer,BLOCK_BUFFER_SIZE,sizeof(block_state));
#else
	NoOperation;
#endif

}


