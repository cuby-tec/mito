/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

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

#ifndef planner_h
#define planner_h



#ifdef kTF7
#include "Block_command.h"
#endif
#include "config.h"

#include "Planner_t.h"


//---------- Defs

#define MAX_TAN_90	3438.0   // 89 50' Значение тангенса для угла близкого к 90

//------------ Vars

extern planner_t pl;

//----------- Function

      
// Initialize the motion plan subsystem
/**
 * Перед выполнением программы загрузить текущее положение инструмента,
 * от которого будет строится дальнейшая траектория.
 */
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
#ifndef KTF7
void plan_buffer_line(float x, float y, float z, float feed_rate, uint8_t invert_feed_rate);
#else
void plan_buffer_line(float x, float y, float z);
void plan_seeek(float x, float y, float z);
#endif

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
//block_t *plan_get_current_block();

// Reset the planner position vector (in steps)
void plan_set_current_position(int32_t x, int32_t y, int32_t z);

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize(int32_t step_events_remaining);

// Reset buffer
void plan_reset_buffer();

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

// Block until all buffered steps are executed
void plan_synchronize();

/**
 * Установление параметров для скорости м1(м/сек)
 */
void setSpeedLevel(block_state* block, float v1);

/**
 * Построение блока для поиска Оси.
 * Param: y - расстояние для перемещения.
 * block - блок-задание на перемещение.
 */
void plan_seekPath(block_state* block, double y);

/**
 * Блок завершения программы.
 */
void plan_end();

/**
 * Перевод инструмента в заданную точку.
 * Построение блока для перемещения в заданную точку.
 * Return: 0 - нет перемещения( нулевое смещение)
 * 1 - есть перемещение.
 */
int plan_MoveToPoint(block_state* block, double_t y );

#endif
