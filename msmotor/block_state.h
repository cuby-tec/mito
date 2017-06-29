//
//
//  Generated by StarUML(tm) C++ Add-In
//
//  @ Project : Untitled
//  @ File Name : block_state.h
//  @ Date : 28.12.2012
//  @ Author : 
//
//


#if !defined(_BLOCK_STATE_H)
#define _BLOCK_STATE_H

#include <inc/typedefs.h>

//  Описатель состояний обработки фаз движения по оси Y.
//  Три состояния: разгон, движение, торможение.
// 
//  Функция перерегулирования. При разгоне будет отставание, при торможении - опережение.
typedef struct block_state_t {
//public:
	//  Индикатор текущего и начального состояния автомата обработки прерываний по оси X.
	byte state;
	uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
	word steps_x;
	word steps_y;
	word steps_z;
	word accelerate_until;
	word decelerate_after;
	word initial_rate;
	word initial_speedLevel;
	word nominal_rate;
	word speedLevel; //  ступень скорости   разгона
	word final_rate;
	word final_speedLevel;
	word step_event_count;
	float tan_theta;		//  Значение тангенса угла наклона отрезка траектории.
//	float tangent_inv; //  Обратное значение тангенса. Для подпрограммы обработки прерываний.
	float nominal_speed;

	/**	  Схема описывает порядок переходов состояний."
	 *
	    Значения состояний:
	    1 - разгон на начальном участке
	    2 - линейное движение
        3 - торможение на конечном участке

        4 - торможение на начальном участке
        5 - линейное движение
	    6 - разгон на конечном участке

        7 - разгон на начальном участке
        8 - линейное движение
        9 - разгон на конечном участке

        10 - торможение на начальном участке
        11 - линейное движение
	    12 — торможение на конечном участке

        13 - разгон на начальном участке
        14 - торможение на конечном участке

        16 - торможение на начальном участке
	    17 - разгон на конечном участке

        24 - Разгон на начальном участке   SEEK
        25 - линейное движение     SEEK
        26 - торможение на конечном участке SEEK



	    Условия перехода определяются значениями
        accelerate_until - это значения по оси X до которого
        автомат находится в состоянии [0], посде чего
        перехдит в состояние с индексом [1].
        По достижении значения decelerate_afetr,
        автомат переходит в состояние с индексом [2].
        Схема обработки начального и конечного участков сегмента.
	  * варианты схем движения: разгон, линейно, разгон
	  * || торможение, линейно, торможение
	  * || разгон, торможение.
	  * и т.д.
	 */
	byte schem[3];

}block_state;

#endif  //_BLOCK_STATE_H