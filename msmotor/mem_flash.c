/*
 * mem_flash.c
 * Company: CUBY
 *  Created on: 23.11.2013
 *      Author: walery
 */


#include "mem_flash.h"
//#include "grbl/settings.h"
//------------ Definition ------


//------------ Variable --------
/**
 *  float steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  float default_feed_rate;
  float default_seek_rate;
  uint8_t invert_mask;
  float mm_per_arc_segment;
  float acceleration;
  float junction_deviation;
  uint8_t flags;  // Contains default boolean settings
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
  uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
  uint8_t decimal_places;
  uint8_t n_arc_correction;
 *  float mm_per_encoder;
 *  uint16 stepmotor_steps;	// (360/1.8)
 *  float rad_acceleration;	// 280//120//287.0 // ( (рад/сек^2)
 *  word initial_rate;		//  Начальное значение делителя.
 *  uint32_t fcnt;//частота работы счётчика
 * Максимальная скорость работы для параметров 0.008mm/step.
 * Максимальное значение делителя 4000
 *  float max_nominal_speed;
 *  float seekSpeed;	// (m/sec)
 *
 *
 */

#pragma location=SEGMENT_STORE
const  settings_t settings ={
		DEFAULT_X_STEPS_PER_MM,DEFAULT_Y_STEPS_PER_MM,DEFAULT_Z_STEPS_PER_MM,
		MICROSTEP, 								// microsteps
		DEFAULT_STEP_PULSE_MICROSECONDS,//pulse_microseconds
		DEFAULT_FEEDRATE, 				//default_feed_rate
		DEFAULT_RAPID_FEEDRATE,			// default_seek_rate
		0,								//invert_mask
		DEFAULT_MM_PER_ARC_SEGMENT,		// mm_per_arc_segment
		DEFAULT_ACCELERATION,			// acceleration
		0,								// junction_deviation
		0,								// flags
		DEFAULT_HOMING_DIR_MASK,		// homing_dir_mask
		DEFAULT_HOMING_FEEDRATE,		// homing_feed_rate
		DEFAULT_HOMING_RAPID_FEEDRATE,	// homing_seek_rate
		DEFAULT_HOMING_DEBOUNCE_DELAY,	//homing_debounce_delay
		DEFAULT_HOMING_PULLOFF,			// homing_pulloff
		DEFAULT_STEPPER_IDLE_LOCK_TIME,	// stepper_idle_lock_time
		DEFAULT_DECIMAL_PLACES,			// decimal_places
		DEFAULT_N_ARC_CORRECTION,		// n_arc_correction
		MM_PER_ENCODER,					// mm_per_encoder
		DEFAULT_STEPMOTOR_STEPS,		// stepmotor_steps
		DEFAULT_RADACCELERATION,		// rad_acceleration
		23964, 							//INITIAL_RATE,	// initial_rate
		DEFAULT_COUNTERR_FREWUENCY,		// fcnt
		MAX_NOMINAL_SPEED,				// max_nominal_speed
		DEFAULT_SEEK_SPEED				// seekSpeed
		};
#ifdef KTF7
settings_t* psettings = (settings_t*)SEGMENT_STORE;
#else
settings_t* psettings = &settings;
#endif
//settings_t* psettings = &settings;

//const  int x @SEGMENT_STORE = 200;

//----------- Function ---------

/**
 * Формальный метод
 * Иначе переменная не объявляется при компоновке.
 */
void init_memflash(){
	if(settings.steps_per_mm[Y_AXIS])
		NoOperation;
}
/*
// Test
void mf_checkState(){
	double_t dd;
	initGRBL();
	if(settings.steps_per_mm[X_AXIS]>0.0){
		dd = psettings->steps_per_mm[X_AXIS];
		dd++;
	}

}
*/
//------------ EOF --------


