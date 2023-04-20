//==============================================================================
// delay_us.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "delay_us.h"

#ifdef MCU

#include "tim.h"
#include "stm32f4xx_hal_tim.h"

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim6,0);             			// set the counter value a 0
	while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) < us); 	// wait for the counter to reach the us input in the parameter

}

#else // SITL

void delay_us(uint16_t us) { }

#endif
