/*
 * Planner_t.h
 * Company: CUBY
 * Project: 052ktf
 *  Created on: 10.12.2012
 *      Author: walery
 */

#ifndef PLANNER_T_H_
#define PLANNER_T_H_

#include <inc/typedefs.h>
#include <stdint.h>
//--- Defs

// Define planner variables
typedef struct {
  int32_t position[3];             // The planner position of the tool in absolute steps. Kept separate
                                   // from g-code position for movements requiring multiple line motions,
                                   // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[3];     // Unit vector of previous path line segment
//  float previous_nominal_speed;   // Nominal speed of previous path line segment
} planner_t;
//---------------- Vars

extern planner_t pl;


//----------------- Functions





#endif /* PLANNER_T_H_ */
