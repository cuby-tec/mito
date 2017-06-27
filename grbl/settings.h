/*
  settings.h - eeprom configuration handling 
  Part of Grbl

  Project: 052KTF

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef settings_h
#define settings_h

#include <math.h>
#include "nuts_bolts.h"

#ifdef Ktf7
#include <cpu_defs.h>
#include <mem_flash.h>
#endif

#include "settings_t.h"
#define GRBL_VERSION "0.8c"

#define TM4


#ifdef TM4  // 175

#define OS_FSYS     80000000
#define timer_devider   2 //8//2 //4

//  Режим работы шагового двигателя: полный, полушаг, 1 4, 1    16 и т.д.
//  Микрошаг
#define MICROSTEP   2

#define DEFAULT_STEPMOTOR_STEPS     (MOTOR_STEPS_PER_REV*MICROSTEP) // (360/1.8)


/**
 * Учтено деление частоты процессора
 * за счёт использования только фронта импульса.
 */
//#define DEFAULT_COUNTERR_FREWUENCY    2000000
#define DEFAULT_COUNTERR_FREWUENCY  OS_FSYS/timer_devider

#define kalf                        0.676
#define _2PI    2*3.141597
//#define alfa                  (float)(_2PI/settings.stepmotor_steps)
//#define alfa                  2*3.141507/DEFAULT_STEPMOTOR_STEPS
#define alfa                    _2PI/DEFAULT_STEPMOTOR_STEPS

#define DEFAULT_RADACCELERATION     400//300//200//280// 120//280 // rad/sec^2

//  Определяет ускорение разгона.
#define INITIAL_RATE                DEFAULT_COUNTERR_FREWUENCY*kalf*sqrt(2*alfa/DEFAULT_RADACCELERATION)

 //29585
#define CO      psettings->initial_rate/kalf

/**
 * Для компенсайии икривления формы трактории вектора скорости требуется
 * ввести поправочный коэффициент для скорости по оси X,
 * уменьшающий расчётное значение. Таким образом S-образная форма
 * траектории вектора будет укладываться в требуемый временной диапазон.
 * Принятое значение в 2% - эмпирическая величина, теорией не подтверждена.
 */
#define COEFFICIENT_FORM        1.2//1.08//1.02//0.98

#define MM_PER_ENCODER  6.0/(360/6)


#endif

#ifdef KTF7



#define APPROX_MAX_CNT	48000

#define MM_PER_ENCODER	6.0/(360/6)


//  Режим работы шагового двигателя: полный, полушаг, 1 4, 1    16 и т.д.
//  Микрошаг
#define MICROSTEP					2

#define DEFAULT_STEPMOTOR_STEPS		(MOTOR_STEPS_PER_REV*MICROSTEP) // (360/1.8)

/**
 * Для компенсайии икривления формы трактории вектора скорости требуется
 * ввести поправочный коэффициент для скорости по оси X,
 * уменьшающий расчётное значение. Таким образом S-образная форма
 * траектории вектора будет укладываться в требуемый временной диапазон.
 * Принятое значение в 2% - эмпирическая величина, теорией не подтверждена.
 */
#define COEFFICIENT_FORM		1.2//1.08//1.02//0.98


#define DEFAULT_RADACCELERATION		400//300//200//280// 120//280 // rad/sec^2


/**
 * Учтено деление частоты процессора
 * за счёт использования только фронта импульса.
 */
//#define DEFAULT_COUNTERR_FREWUENCY	2000000
#define DEFAULT_COUNTERR_FREWUENCY	OS_FSYS/timer_devider

/**
 * Константа для вычисления скорости перемещения по оси X
 */
//  Для мм/мин
#define SPEED_X_CONSTANT (DEFAULT_COUNTERR_FREWUENCY*6.0)
//  Для мм/мин
#define SPEED_X_CONSTANT_MM		(DEFAULT_COUNTERR_FREWUENCY/10)

//Для м/сек
#define SPEED_X_CONSTANT_M		(DEFAULT_COUNTERR_FREWUENCY/10/1000)

// for display speed average для перевода в мм/мин
#define SPEED_AVERAGE_M			(60.0*1000.0)

#define kalf						0.676

#define _2PI	2*3.141597
//#define alfa					(float)(_2PI/settings.stepmotor_steps)
//#define alfa					2*3.141507/DEFAULT_STEPMOTOR_STEPS
#define alfa					_2PI/DEFAULT_STEPMOTOR_STEPS

// DEFAULT_COUNTERR_FREWUENCY*kalf*sqrt(2*alfa/DEFAULT_RADACCELERATION)// DEFAULT_RADACCELERATION
//#define INITIAL_RATE				DEFAULT_COUNTERR_FREWUENCY*kalf*sqrt(2*alfa/settings.rad_acceleration)//20000 //  Определяет ускорение разгона.
#define INITIAL_RATE				DEFAULT_COUNTERR_FREWUENCY*kalf*sqrt(2*alfa/DEFAULT_RADACCELERATION)	//  Определяет ускорение разгона.

 //29585
#define CO		psettings->initial_rate/kalf

#endif

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 5

// Define bit flag masks for the boolean settings in settings.flag.
#define BITFLAG_REPORT_INCHES      bit(0)
#define BITFLAG_AUTO_START         bit(1)
#define BITFLAG_INVERT_ST_ENABLE   bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3)
#define BITFLAG_HOMING_ENABLE      bit(4)

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future 
// developments.
#define EEPROM_ADDR_GLOBAL 1
#define EEPROM_ADDR_PARAMETERS 512
#define EEPROM_ADDR_STARTUP_BLOCK 768

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)


//extern settings_t settings;

//------- Function
void settings_reset(bool reset_all);

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

// A helper method to set new settings from command line
uint8_t settings_store_global_setting(int parameter, float value);

// Stores the protocol line variable as a startup line in EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable
uint8_t settings_read_startup_line(uint8_t n, char *line);

// Writes selected coordinate data to EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// Reads selected coordinate data from EEPROM
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

#endif
