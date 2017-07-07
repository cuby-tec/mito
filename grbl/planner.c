/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  Copyright (c) 2011 Jens Geisler  
  
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

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */
#ifdef KtF7
#include <rtos.h>
#include <blockQuee.h>
#include <mempool.h>
#else
#include <msmotor/mem_flash.h>
#include <Ktf_model.h>
#endif

#include "grbl/BlockQuee.h"
#include "msmotor/ms_model.h"
#include <orderlyTask.h>

//#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include "grbl/planner.h"
#include "grbl/nuts_bolts.h"
//#include "stepper.h"
#include "grbl/settings.h"
//#include <pindef.h>
//#include "config.h"
//#include "protocol.h"

//#include "gcode.h"





//------  Defs

#ifdef TM4
#define OS_free free
#define OS_malloc   malloc
#endif


//#define _2PI	2*3.141597
#define sec_min		60

//280//120//287.0 // (рад/сек^2)
//#define rad_acceleration		psettings->rad_acceleration
#define rad_acceleration		ktf_model.rad_acceleration
//#define alfa					(float)(_2PI/psettings->stepmotor_steps)



#define HOME_SPEED		6 // mm/sec

// -> ms_model.h
//#define MIN_MOVEON_X		0.4/60 // m/sec   6.6// mm/sec

//#define accel_Y			280 // mm/sec^2
//psettings->acceleration

/*
#define check_state_go		if(block->steps_y){\
		if(block->accelerate_until == 0){\
			block->accelerate_until++;\
		}\
		if(block->decelerate_after = block->steps_y){\
			block->decelerate_after--;\
		}\
	}\
*/


//------------- Vars -------
// Скорость на предыдущем участке
static	float vn = 0;

planner_t pl;


//-------- Function


/**
 * Определить количество шагов и направление перемещения до следующей точки траектории.
 */
static int plan_setSteps(block_state* block, float x, float y, float z){
	int32_t* target = (int32_t*) OS_malloc(sizeof(int32_t[3]));

	if(!target)return (0);

	memset(block,0,sizeof(block_state));


	// Calculate target position in absolute steps
	target[X_AXIS] = (int32_t)(x*psettings->steps_per_mm[X_AXIS]);
	target[Y_AXIS] = (int32_t)(y*psettings->steps_per_mm[Y_AXIS]);
	target[Z_AXIS] = (int32_t)(z*psettings->steps_per_mm[Z_AXIS]);

	// Number of steps for each axis
	block->steps_x = labs(target[X_AXIS]-pl.position[X_AXIS]);
	block->steps_y = labs(target[Y_AXIS]-pl.position[Y_AXIS]);
	block->steps_z = labs(target[Z_AXIS]-pl.position[Z_AXIS]);

	block->step_event_count =  block->steps_y;


	pl.previous_unit_vec[X_AXIS] = x;
	pl.previous_unit_vec[Y_AXIS] = y;
	pl.previous_unit_vec[Z_AXIS] = z;


	// Bail if this is a zero-length block
	if (block->step_event_count == 0){
		// Update planner position
		memcpy(pl.position, target, sizeof(int32_t[3])); // pl.position[] = target[]
		OS_free(target);
//		OS_free(block);
		return (0);
	}
#ifdef KTF7
	if(target[Y_AXIS]>pl.position[Y_AXIS])
		block->direction_bits |= DIRECTION_MASK;
	else
		block->direction_bits &= ~DIRECTION_MASK;
	block->direction_bits |= STEP_MASK;
#endif

	// Update planner position
	memcpy(pl.position, target, sizeof(int32_t[3])); // pl.position[] = target[]

	OS_free(target);

	return (1);
}



/*                             STEPPER RATE DEFINITION                                              
                                     +--------+   <- nominal_rate
                                    /          \                                
    nominal_rate*entry_factor ->   +            \                               
                                   |             + <- nominal_rate*exit_factor  
                                   +-------------+                              
                                       time -->                                 
  
*/

/*                            PLANNER SPEED DEFINITION                                              
                                     +--------+   <- current->nominal_speed
                                    /          \                                
         current->entry_speed ->   +            \                               
                                   |             + <- next->entry_speed
                                   +-------------+                              
                                       time -->                                 
*/                                                                              


//#define CO		psettings->initial_rate/0.676

static void planner_recalculate(block_state* prev, block_state* curr){
	int16 dlevel,d2;
	word meanlevel;
	word tmp_rate;
	if(prev->speedLevel == curr->speedLevel) return;

	meanlevel = ((dword)prev->nominal_rate + curr->nominal_rate)/2;

	if(prev->speedLevel>curr->speedLevel){
	    // Снижение скорости
		dlevel = prev->speedLevel - curr->speedLevel;
		d2 = dlevel/2;
		dlevel -= d2;
		prev->decelerate_after = prev->steps_y - d2;
		curr->accelerate_until = dlevel;	// todo if dlevel/2 > accelerate_until
		prev->schem[2] = 3;
		curr->schem[0] = 4;

		d2 = prev->speedLevel-d2;
		if(d2){
			prev->final_speedLevel = d2;
			tmp_rate = (word)(CO)*(sqrtf(d2+1)-sqrtf(d2));
			prev->final_rate = (tmp_rate>meanlevel)?(tmp_rate):(meanlevel);
		}
		dlevel += curr->speedLevel;
		if(dlevel){
			curr->initial_speedLevel = dlevel;
			tmp_rate = (word)CO*(sqrtf(dlevel+1)-sqrtf(dlevel));
			curr->initial_rate = (tmp_rate>meanlevel)?(tmp_rate):(meanlevel);

		}
	}else{
	    // Увеличение скорости
		dlevel = curr->speedLevel - prev->speedLevel;
		word rest = prev->steps_y - prev->decelerate_after;
		if(rest>dlevel){
			d2 = dlevel/2;
		}else{
			d2 = rest;
		}
			dlevel -= d2;

		prev->decelerate_after = prev->steps_y - d2;
		curr->accelerate_until = dlevel;	// todo if dlevel/2 > accelerate_until
		prev->schem[2] = 6;
		curr->schem[0] = 1;
		d2 +=prev->speedLevel;
		if(d2){
			prev->final_speedLevel = d2;
			prev->final_rate = (word)CO*(sqrtf(d2+1)-sqrtf(d2));
		}
		dlevel = curr->speedLevel - dlevel;
		if(dlevel){
			curr->initial_rate = (word)CO*(sqrtf(dlevel+1)-sqrtf(dlevel));
			curr->initial_speedLevel = dlevel;
		}
	}
	//------------------------- check state go

}


void plan_init(){
	pl.position[X_AXIS] = ktf_model.pozX_cnt;
	pl.position[Y_AXIS] = ktf_model.pozY_cnt;
	pl.position[Z_AXIS] = 0;
}




//Начальная скорость участка. Для первой линии она равна нулю.
// Для последующих - равна конечной скорости предыдущего участка.
float getFlatSpeed(){
//	return MIN_MOVEON_X; //  м/сек
	return sts.speedX*COEFFICIENT_FORM;
}

/**
 * Блок завершения программы.
 */
void plan_end(){
	block_state* block = getBlock();
	if(block){
		memset(block,0,sizeof(block_state));
		block->schem[0] = MS_BLOCK_END; // M30 - end program;
	}else{
		NoOperation; // todo Error getBlock
	}

}

void setSpeedLevel(block_state* block, float v1){
    uint32_t cnt;

    //Радиальная скорость  =E33*_2PI/(sy)*(1000/sec_min)
//	float rad_y_speed = v1*_2PI*psettings->steps_per_mm[Y_AXIS]/sec_min;
	block->nominal_speed = v1;
//	float rad_y_speed = v1*_2PI*psettings->steps_per_mm[Y_AXIS];
	double_t rad_y_speed = v1*_2PI/SCRWE_PITCH*1000;	//  рад/сек
    double_t freq_priv = rad_y_speed;


	// ступень скорости
	double_t dt = rad_y_speed;
	dt *= rad_y_speed;
	dt /= alfa;
	dt /= rad_acceleration;
	dt /=2;

//	block->speedLevel = (rad_y_speed*rad_y_speed)/alfa/rad_acceleration/2 ;
	block->speedLevel = dt;

	// Частота привода Y
//	double_t freq_priv = rad_y_speed/alfa;
//	double_t freq_priv = rad_y_speed;
	freq_priv /= alfa;


	// Делитель для номинальной скорости
	cnt = psettings->fcnt/freq_priv;
	if(cnt < 65535)
		block->nominal_rate = cnt;
	else
		block->nominal_rate = 65535;

	block->final_speedLevel = block->speedLevel;

	block->initial_rate = psettings->initial_rate;
	block->final_rate = block->initial_rate;	// 20000 - default
}


//void plan_seekZero(block_state* block, byte dir){
void plan_seekPath(block_state* block, double_t y){

	block->steps_y = (int32_t)(y*psettings->steps_per_mm[Y_AXIS]);
#ifdef KTF7
	block->direction_bits |= STEP_MASK;
#else
	block->direction_bits |= 0xFF;  // debug
#endif

	setSpeedLevel(block, psettings->seekSpeed);
	if(block->speedLevel>=block->steps_y){
		block->accelerate_until = block->steps_y/2;
		block->decelerate_after = block->steps_y/5;
	}else{
		block->accelerate_until = block->speedLevel;
		block->decelerate_after = block->steps_y - block->speedLevel;
	}
	block->schem[0] = 24;
	block->schem[1] = 25;
	block->schem[2] = 26;
}



int plan_MoveToPoint(block_state* block, double_t y ){
    // Перевод инструмента в заданную точку.
	int result = 0;
	plan_init();
	if(plan_setSteps(block, 0, y, 0)){
		NoOperation;
		setSpeedLevel(block, psettings->seekSpeed);
		if(block->speedLevel>=block->steps_y){
			block->accelerate_until = block->steps_y/2;
			block->decelerate_after = block->steps_y/5;
		}else{
			block->accelerate_until = block->speedLevel;
			block->decelerate_after = block->steps_y - block->speedLevel;
		}

		block->schem[0] = 24;
		block->schem[1] = 25;
		block->schem[2] = 26;
		result++;
	}
	return result;
}

//----- by Program G-CODE


/**
 * Переход по оси Y с номинальной скоростью.
 * param: float y - координаты цели.
 */
void plan_seeek(float x, float y, float z){
//	int32_t* target;
#ifdef KTF7
	block_state *block = (block_state*)OS_malloc(sizeof(block_state));
#else
	block_state *block = (block_state*)malloc(sizeof(block_state));
#endif
//	float* delta_mm;

	if(!plan_setSteps(block, x,y,z)){
		OS_free(block);
		return;
	}

	// Начальная скорость участка
	//	float v0;
	// номинальная скорость участка.
//	float v1 = HOME_SPEED;
//	block->nominal_speed = HOME_SPEED;// v1
//------------------
	setSpeedLevel(block, psettings->seekSpeed);

	if(block->speedLevel>=block->steps_y){
		block->accelerate_until = block->steps_y/2;
		block->decelerate_after = block->steps_y/5;
	}else{
		block->accelerate_until = block->speedLevel;
		block->decelerate_after = block->steps_y - block->speedLevel;
	}

	block->schem[0] = 24;
	block->schem[1] = 25;
	block->schem[2] = 26;

	// check change states
//	check_state_go;
//---------------------
	block_state* block_q = getBlock();
	if(block_q){
		memcpy(block_q,block,sizeof(block_state));
		OS_free(block);
		if(ktf_model.block_counter == 1)
			MS_COMMAND_SEMA_SET;	// todo Модальные команды
		//	MS_COMMAND_SEMA_IE;
	}else{
		NoOperation; // todo getBlock error
	}
}


/**
 * plan_buffer_line
 */
void plan_buffer_line(float x, float y, float z, float e, uint8_t inv) {
	int32_t* target;
	// Prepare to set up new block
#ifdef GET_BLOCK
	block_t *block = getBlock();
#else
//	block_t *block = (block_t*)OS_malloc(sizeof(block_t));
#ifdef KtF7
	block_state* block = (block_state*)OS_malloc(sizeof(block_state));
#else
	block_state* block = (block_state*)malloc(sizeof(block_state));
#endif
	block_state* block_q;
//	block_t *block_q;
#endif

	///////////////////////   V2   //////////////////////
#define V2
	float* delta_mm;

	memset(block,0,sizeof(block_state));

	target = (int32_t*) OS_malloc(sizeof(int32_t[3]));
	// Calculate target position in absolute steps
	target[X_AXIS] = (int32_t)(x*psettings->steps_per_mm[X_AXIS]);
	target[Y_AXIS] = (int32_t)(y*psettings->steps_per_mm[Y_AXIS]);
	target[Z_AXIS] = (int32_t)(z*psettings->steps_per_mm[Z_AXIS]);

	// Number of steps for each axis
	block->steps_x = labs(target[X_AXIS]-pl.position[X_AXIS]);
	block->steps_y = labs(target[Y_AXIS]-pl.position[Y_AXIS]);
	block->steps_z = labs(target[Z_AXIS]-pl.position[Z_AXIS]);
//	block->step_event_count = max(block->steps_x, block->steps_y);
	block->step_event_count = block->steps_x;
	// Bail if this is a zero-length block
	// todo только рабочая обработка. Без Дом или Выход на координаты.
	if (block->step_event_count == 0){
		memcpy(pl.position, target, sizeof(int32_t[3]));
		pl.previous_unit_vec[X_AXIS] = x;
		pl.previous_unit_vec[Y_AXIS] = y;
		pl.previous_unit_vec[Z_AXIS] = z;

		OS_free(target);
		OS_free(block);
		return;
	}

#ifdef KTF7
	if(target[Y_AXIS]>pl.position[Y_AXIS])
		block->direction_bits |= DIRECTION_MASK;
	else
		block->direction_bits &= ~DIRECTION_MASK;
	if(!block->steps_y)
		block->direction_bits &= ~STEP_MASK;
	else
		block->direction_bits |= STEP_MASK;
#endif
	// Update planner position
	memcpy(pl.position, target, sizeof(int32_t[3])); // pl.position[] = target[]
	OS_free(target);

	// Compute path vector in terms of absolute step target and current positions
	//  float delta_mm[3];
	delta_mm = (float*) OS_malloc(sizeof(float[3]));
	delta_mm[X_AXIS] = x - pl.previous_unit_vec[X_AXIS];
	delta_mm[Y_AXIS] = y - pl.previous_unit_vec[Y_AXIS];
	delta_mm[Z_AXIS] = z - pl.previous_unit_vec[Z_AXIS];

	if(delta_mm[X_AXIS]!=0){
		block->tan_theta = (delta_mm[Y_AXIS]/delta_mm[X_AXIS]);
//		block->tan_theta = delta_mm[Y_AXIS]/delta_mm[X_AXIS];
	}else{
		if(delta_mm[Y_AXIS]>0)
			block->tan_theta = MAX_TAN_90;// 89 50' Значение тангенса для угла близкого к 90
		else
			block->tan_theta = -MAX_TAN_90;// -89 50' Значение тангенса для угла близкого к -90
	}

	OS_free(delta_mm);

	pl.previous_unit_vec[X_AXIS] = x;
	pl.previous_unit_vec[Y_AXIS] = y;
	pl.previous_unit_vec[Z_AXIS] = z;

	// Продольная скорость участка
	float v0;
	// номинальная, поперечная скорость участка - Y.
	float v1;

	if(block->tan_theta){
		v0 = getFlatSpeed();
		v1 = fabs(v0*block->tan_theta);
		setSpeedLevel(block,v1);
		if(block->speedLevel>=block->steps_y){
			block->accelerate_until = block->steps_y/2;
			block->decelerate_after = block->steps_y/5;
			block->schem[0] = 1;
			block->schem[1] = 3;
			block->schem[2] = 3;
		}else{
			block->accelerate_until = block->speedLevel;
			block->decelerate_after = block->steps_y - block->speedLevel;
			block->schem[0] = 1;
			block->schem[1] = 2;
			block->schem[2] = 3;
		}
	}

//	check_state_go;

	block_q = getBlock();
	if(block_q){
		memcpy(block_q,block,sizeof(block_state));
		// todo planner_recalculate
		// Если есть предшествующие блоки и не происходит смены направления,
		// то пересчитать длительность и направление смежных участков (сегментов).
#ifdef KtF7
		if(OS_MEMF_GetNumFreeBlocks(&cmdPool)<BLOCK_BUFFER_SIZE-1){
			uint8_t prevBlockNumber = prev_block_index(block_buffer_head);
			uint8_t a = (cmdQuee[prevBlockNumber])->direction_bits;
			a ^= block_q->direction_bits;
			//		if((block_q->direction_bits&DIRECTION_MASK) ^ (cmdQuee[prevBlockNumber].direction_bits&DIRECTION_MASK))
			if(((~(block_q->direction_bits ^ (cmdQuee[prevBlockNumber])->direction_bits))&DIRECTION_MASK)
					&&(block_q->direction_bits&STEP_MASK))
				//		if((~a)&DIRECTION_MASK)
				planner_recalculate(cmdQuee[prevBlockNumber],cmdQuee[block_buffer_head]);
			NoOperation;
		}
#endif
#ifndef GET_BLOCK
		OS_free(block);
#endif
	}else{
		NoOperation; // todo Error getBlock
	}


}


// Reset the planner position vector (in steps). Called by the system abort routine.
void plan_set_current_position(int32_t x, int32_t y, int32_t z)
{
  pl.position[X_AXIS] = x;
  pl.position[Y_AXIS] = y;
  pl.position[Z_AXIS] = z;
}

// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.
void plan_cycle_reinitialize(int32_t step_events_remaining) 
{
  block_t *block = {0};// todo plan_cycle_reinitialize
//-  block_t *block = &block_buffer[block_buffer_tail]; // Point to partially completed block
  
  // Only remaining millimeters and step_event_count need to be updated for planner recalculate. 
  // Other variables (step_x, step_y, step_z, rate_delta, etc.) all need to remain the same to
  // ensure the original planned motion is resumed exactly.
  block->millimeters = (block->millimeters*step_events_remaining)/block->step_event_count;
  block->step_event_count = step_events_remaining;
  
  // Re-plan from a complete stop. Reset planner entry speeds and flags.
  block->entry_speed = 0.0;
  block->max_entry_speed = 0.0;
  block->nominal_length_flag = false;
  block->recalculate_flag = true;
#if defined  KTF7 || !defined  TM4
  planner_recalculate();
#endif
}
