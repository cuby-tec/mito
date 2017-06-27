/*
 * settings_t.h
 * Company: CUBY
 *  Created on: 23.11.2013
 *      Author: walery
 */

#ifndef SETTINGS_T_H_
#define SETTINGS_T_H_


#include "inc/typedefs.h"
#include <stdint.h>
//---------- Definition -------
#define TM4
#define KTF7_no

// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards)
typedef struct {
  float steps_per_mm[3];
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
  uint16 homing_debounce_delay;
  float homing_pulloff;
  uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
  uint8_t decimal_places;
  uint8_t n_arc_correction;
//  uint8_t status_report_mask; // Mask to indicate desired report data.
#if defined KTF7 || defined TM4
  //  Расстояние, прожодимое за оодин период сигналов энкодера.
  // Определяется как шаг винта делённый на количество импульсов энкодера за один оборот винта.
  float mm_per_encoder;
  uint16 stepmotor_steps;	// (360/1.8)
  float rad_acceleration;	// 280//120//287.0 //  (рад/сек^2)
  word initial_rate;		//  Начальное значение делителя.
  //частота работы счётчика
  uint32_t fcnt;
  //  Максимальная скорость работы для параметров 0.008mm  / step.
  //  Максимальное значение делителя  4000
  float max_nominal_speed;
  float seekSpeed;	// (m/sec)
//  float approx_dec;
//  float approx_acc;
//  uint8_t appropx_shift; //  Коэффициент полинома аппроксимации.
#endif
} settings_t;

//----------- Vars ----------


//---------- Function --------



#endif /* SETTINGS_T_H_ */
