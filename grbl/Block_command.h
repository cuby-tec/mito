/*
 * Block_command.h
 * Company: CUBY
 * Project: 052ktf
 *  Created on: 10.12.2012
 *      Author: walery
 */

#ifndef BLOCK_COMMAND_H_
#define BLOCK_COMMAND_H_

#include <stdint.h>
#include <math.h>
//#include <inc/typedefs.h>
#include "msmotor/block_state.h"
//--- Defs


// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {

  // Fields used by the bresenham algorithm for tracing the line
  uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  uint32 steps_x, steps_y, steps_z; // Step count along each axis
  int32  step_event_count;          // The number of step events required to complete this block

  // Fields used by the motion planner to manage acceleration
  float nominal_speed;               // The nominal speed for this block in mm/min
  float entry_speed;                 // Entry speed at previous-current block junction in mm/min
  float max_entry_speed;             // Maximum allowable junction entry speed in mm/min
  float millimeters;                 // The total travel of this block in mm
  float tan_theta;					//  Тангенс угла наклона отрезка к оси X
  uint8_t recalculate_flag;           // Planner flag to recalculate trapezoids on entry junction
  uint8_t nominal_length_flag;        // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  uint16 initial_rate;              // The step rate at start of block
  uint16 final_rate;                // The step rate at end of block
//  	int16 rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
  uint32 accelerate_until;          // The index of the step event on which to stop acceleration
  uint32 decelerate_after;          // The index of the step event on which to start decelerating
  uint16 nominal_rate;              // The nominal step rate for this block in step_events/minute

} block_t;

//---------------- Vars



//----------------- Functions





#endif /* BLOCK_COMMAND_H_ */
