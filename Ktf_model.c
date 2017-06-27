/*
 * Ktf_model.c
 * Company: CUBY
 * Project: 052ktf
 *  Created on: 01.12.2012
 *      Author: walery
 */


#include "Ktf_model.h"
#ifdef KtF7
#include <keypad.h>
#include <calmLib.h>
#endif

#include "grbl/nuts_bolts.h"
//#include "settings.h"
#include "msmotor/mem_flash.h"


//--- Defs

#define ENDER_TIMEOUT		16


//----------------- Vars 
#ifdef KTF7
#pragma object_attribute = __no_init
Ktf_model ktf_model;
#ifdef ENDER_IN_KTF
OS_TIMER enderTimer;
#endif
#else
Ktf_model ktf_model;
#endif  // KTF7
//------------------ Functions


/**
 * По заданному значению координаты Y выставляется счётчик тактов шагового привода.
 * Param: pos - координата y в миллиметрах.
 */
void setY_counter(double_t pos){
	_posY = pos*psettings->steps_per_mm[Y_AXIS];
}

void setX_counter(double_t pos){
	_posX = pos*psettings->steps_per_mm[X_AXIS];
}




void ktf_pos_Reset(){
	/**
"    * Инструмент находится в состоянии "   Дом
"    * Переход в состояние Дом выполняется вручную:"
"    *  на станке и на контроллере 052."
	 */
//	ktf_model.pozX = 0;

	/**
	 * Положение Y должно находиться в состоянии Дом
	 * перед запуском программы.
	 */
//	ktf_model.pozY = 0; // todo pozY = 0; ?

	ktf_model.block_counter = 0;
	ktf_model.state = ekt_FREE; //  0- Выполнение программы, 1 - сканер данных
	ktf_model.quality = eq_Dirty;
}

void ktf_reset(){
	ktf_model.maxX = 0;
	ktf_model.maxY = 0;
	ktf_model.minX = 1e6;
	ktf_model.minY = ktf_model.minX;
	ktf_model.refresh++;
}


void updateDimentions(){
	ktf_model.maxX_cnt = ktf_model.maxX*psettings->steps_per_mm[X_AXIS];
	ktf_model.minX_cnt = ktf_model.minX*psettings->steps_per_mm[X_AXIS];
}

void scan_dimention(float x, float y){
	ktf_model.maxX = max(ktf_model.maxX,x);
	ktf_model.maxY = max(ktf_model.maxY,y);
	if(x!=0){
		ktf_model.minX = min(ktf_model.minX,x);
		ktf_model.minY = min(ktf_model.minY,y);
	}
}

#ifdef KTF7

byte ktf_isZeroOn(){
	return (ENDER_ZERO_IN>0)?(true):(false);
}


byte krf_isHomeOn(){
	return (ENDER_HOME_IN>0)?(true):(false);
}

#ifdef ENDER_IN_KTF
void restoreRnderIE(){
	ENDER_PORT_IFG &= ~ENDER_MASK;
	ENDER_PORT_IE |= ENDER_MASK;
}
#endif
void ktf_initKtfModel(){
	//	ktf_model restore
	//	restore((byte*)&ktf_model,sizeof(Ktf_model));
	memcpy(&ktf_model,(byte*)SEGMENT_B,sizeof(Ktf_model));
	byte crc = msgCRC((byte*)&ktf_model,sizeof(Ktf_model));
	if(crc != CRC_LIB){
		//  Нет сохранённых данных.
		//		OS_Error(388);
		memset(&ktf_model,0,sizeof(Ktf_model));
	}

	ktf_model.hw_sd &= ~(FF_HW_OPEMFILE|FF_MOUNT_SD);

	ktf_model.refresh++;
#ifdef ENDER_IN_KTF
	OS_CreateTimer(&enderTimer,restoreRnderIE,ENDER_TIMEOUT);
#endif
}

/**
 * Port 2 interrupt
 * Для версии 2(RS232) нормально-замкнутое состояние отображается 0,
 * а разомкнутое состояние - 1.
 * И для этой версии платы ENDER_SWITCH_ON должен быть определён как 0.
 */
//#pragma vector = PORT2_VECTOR
//__interrupt void Port2_ISR(void);
__interrupt void Port2_ISR(void){
	OS_EnterInterrupt();
	if(ENDER_HOME_IFG){
		ENDER_HOME_IE = 0;
		ENDER_HOME_IFG = 0;
//		STOP_Y;
#ifdef ENDER_IN_KTF
		OS_RetriggerTimer(&enderTimer);
#endif
		if(ENDER_HOME_IES == ENDER_SWITCH_ON){
			ENDER_HOME_IES = ~ENDER_SWITCH_ON;
			OS_SignalEvent(ev_EnderHome,&keypad);
		}
		else{
			ENDER_HOME_IES = ENDER_SWITCH_ON;
			OS_SignalEvent(ev_EnderHomeFin,&keypad);
		}
		NoOperation;
	}
	if(ENDER_ZERO_IFG){
		ENDER_ZERO_IFG = 0;
//		STOP_Y;
#ifdef ENDER_IN_KTF
		OS_RetriggerTimer(&enderTimer);
#endif
		if(ENDER_ZERO_IES == ENDER_SWITCH_ON){
			ENDER_ZERO_IES = ~ENDER_SWITCH_ON;
			OS_SignalEvent(ev_EnderZero,&keypad);
		}
		else{
			ENDER_ZERO_IES = ENDER_SWITCH_ON;
			OS_SignalEvent(ev_EnderZeroFin,&keypad);
		}
		NoOperation;
	}
	ENDER_PORT_IE &=~ENDER_MASK;// Restore in timer
	OS_RetriggerTimer(&enderTimer);
	OS_LeaveInterrupt();
}

#endif // Ktf7





