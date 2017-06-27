/*
 * Ktf_model.h
 * Company: CUBY
 * Project: 052ktf
 *  Created on: 01.12.2012
 *      Author: walery
 */

#ifndef KTF_MODEL_H_
#define KTF_MODEL_H_


#include "inc/typedefs.h"
//#include <pindef.h>
#include "msmotor/ms_model.h"
//#include <menu_common.h>

//#include <tFatfs.h>
//#define KTF_FILENAME_LENGTH     11  // from tFatfs.h
//--- Defs

#include <math.h>


/**
 * Задержка включения прерывания по концевикам выполняется таймером.
 */
#define ENDER_IN_KTF

//int16 _posX,_posY;
#define _posY		ktf_model.pozY_cnt
#define  _posX		ktf_model.pozX_cnt

/**
 * Флаги байта hw_sd - Состояние файла, жлеза и карты памяти.
 */

#define FF_HW_OPEMFILE		(1)		// Флаг открытия файла.
#define FF_HW_SD			(1<<1)	// флаг карты памяти.
#define FF_MOUNT_SD			(1<<2)	// Флаг монтирования тома

enum eQuality{
	eq_Dirty,eq_Clear
};

enum SyncMode{
    eSyncMode_ms, eAsyncMode_ms
};

enum kt_eStates{
	ekt_FREE, ekt_SEEK, ekt_WORK, ekt_SCAN, ekt_Zero, ekt_Home,
	ekt_CalibreZero, ekt_CalibreZeroFin, ekt_CalibreHome, ekt_CalibreHomeFin, ekt_XposCalibre,
	ekt_MoveHome, ektMoveHome_fin, // Состояния для перемещения в положение Дом.
	ekt_MoveHome_err, // При движении вДом сработвл датчик концевика Дом.
	ekt_WorkHomeEnder, ekt_WorkAxisEnder, ekt_EnderStop
	, ekt_MoveAt
	, ekt_EnderHome_err, ekt_EnderZero_err	// Ошибка исполнения: концевой даичмк.
	, ekt_WorkBtnStop // Останов программы по кнопке
	, ekt_File
	, ekt_SetSync // Установка метода синхронизации обработки движения.
	, ekt_SetRadialAccel // Установка велияины радиального ускорения.
};

typedef struct {

//	int32_t position[3];	// The planner position of the tool in absolute steps. Kept separate
							// from g-code position for movements requiring multiple line motions,
							// i.e. arcs, canned cycles, and backlash compensation.
//	float previous_unit_vec[3];     // Unit vector of previous path line segment

	byte 			hw_sd;
	byte 			refresh;	// Флаг изменения данных для отображения
//	float	pozX;
//	float	pozY;
	int32			pozX_cnt;	// Счётчик положения X: положительные или отрицательные значения.
	int32			pozY_cnt;	// Счётчик положения Y: положительные или отрицательные значения.
#ifdef TFATFS_H_
	char 			file[KTF_FILENAME_LENGTH+1];
#endif
	enum SyncMode	sync;		// Способ обработки блоков движения: 0 - сихронный, 1 - асинхронный.
	word 			block_counter;	// Счётчик загруженных блоков - признак завершения выполнения программы.
	word 			blockNumber;	// Номер текущего(исполняемого) блока.
	enum kt_eStates state;  // Состояние: 0 - свободное движение, 1 - сканирование границ изделия,2 - работа,
	float 			maxX;	// Границы изделия
	int32 			maxX_cnt;
	float 			minX;	// Границы изделия
	int32 			minX_cnt;
	float 			maxY;	// Границы изделия
	float 			minY;	// Границы изделия
	double_t 		calibre_zero;
	double_t 		calibre_Home;
	float			rad_acceleration;
	word			initial_rate;
	enum eQuality 	quality;	// Указатель направления движения: черновой/чистовой.
	byte 			crc;		// Контрольная сумма набора данных.
	byte			level_index;
#ifdef TFATFS_H_
	char			path[PATH_LEVELS][PATH_SIZE]; //path[PATH_LENGTH+1];
#endif
}Ktf_model;

//---------------- Vars

extern Ktf_model ktf_model;

//----------------- Functions

/**
 * Установка МИН и МАКС в исходное значение {0,1e6}.
 */
void ktf_reset();

/**
 * Установка положения
 */
void ktf_pos_Reset();

/**
 * Поиск границ изделия
 */
void scan_dimention(float x, float y);

/**
 * Установка счётчиков положения.
 */
void updateDimentions();

/**
 * Восстановление обработки прерываний Концевика
 * по дребезгу.
 */
void restoreRnderIE();


void ktf_initKtfModel();

/**
 * Проверка условия нахождения концевого датчика Ось в состоянии замкнуто.
 * true - если замкнуто.
 */
byte ktf_isZeroOn();

/**
 * Проверка условия нахождения концевого датчика Дом в состоянии замкнуто.
 * true - если замкнуто.
 */
byte krf_isHomeOn();




/**
 * По заданному значению координаты Y выставляется счётчик тактов шагового привода.
 * Param: pos - координата y в миллиметрах.
 */
void setY_counter(double_t pos);

void setX_counter(double_t pos);


//------- Interrupt handler
#ifdef KTF7
#pragma vector = PORT2_VECTOR
__interrupt void Port2_ISR(void);
#endif


#endif /* KTF_MODEL_H_ */
