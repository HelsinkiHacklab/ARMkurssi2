/*
 * hbridge.cpp
 *
 *  Created on: 23.3.2021
 *      Author: martti
 */

#include "hbridge.h"

namespace H_bridge {

hbridge::hbridge(TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef* _in_A_port, uint16_t _in_A_pin, GPIO_TypeDef* _in_B_port, uint16_t _in_B_pin) {
	htim = _htim;
	channel = _channel;
	in_A_port = _in_A_port;
	in_A_pin = _in_A_pin;
	in_B_port = _in_B_port;
	in_B_pin = _in_B_pin;

	commandState = state_stop;
	actualState = state_stop;

	setSpeed = 500;
	accIncrement = 5.0/1000.0;
}

hbridge::~hbridge() {
	// TODO Auto-generated destructor stub
}

void hbridge::setSpeedReference( uint16_t _setSpeed ) {
	setSpeed = _setSpeed;
}


void hbridge::setAccReference(float _setAcc ) {
	accIncrement = _setAcc / 1000.0;
}

void hbridge::stateMachine() {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, actualState ? GPIO_PIN_SET : GPIO_PIN_RESET );
	switch ( commandState ) {
	case state_forward: {
		switch ( actualState ) {
		case state_stop: {
			HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Start(htim, channel);
			actualState = state_forward;
			break;
		}
		case state_forward: {
			pwmSpeed = iterate( setSpeed, &currentSpeed );
			break;
		}
		case state_reverse: {
			pwmSpeed = iterate( 0, &currentSpeed );
			if (pwmSpeed == 0 ) {
				HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
				actualState = state_forward;
			}
			break;
		}
		}
		break;
	}
	case state_reverse: {
		switch ( actualState ) {
		case state_stop: {
			HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Start(htim, channel);
			actualState = state_reverse;
			break;
		}
		case state_reverse: {
			pwmSpeed = iterate( setSpeed, &currentSpeed );
			break;
		}
		case state_forward: {
			pwmSpeed = iterate( 0, &currentSpeed );
			if (pwmSpeed == 0 ) {
				HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_SET);
				actualState = state_reverse;
			}
			break;
		}
		}
		break;
	}
	case state_stop: {
		if ( actualState != state_stop ) {
			pwmSpeed = iterate( 0, &currentSpeed );
			if (pwmSpeed == 0 ) {
				HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(htim, channel);
				actualState = state_stop;
			}
		}
		break;
	}
	case state_cutoff: {
		HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(htim, channel);
		commandState = state_stop;
		actualState = state_stop;
		break;
	}
	case state_brake: {
		if ( actualState != state_brake ) {
			delayCounter = 1000;
			actualState = state_brake;
			HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
			pwmSpeed = 1000;
		}
		if ( --delayCounter == 0 ) {
			HAL_TIM_PWM_Stop(htim, channel);
			commandState = state_stop;
			actualState = state_stop;
		}
		break;
	}
	default: {
	}
	}
	htim->Instance->CCR3 = pwmSpeed;
}


uint16_t hbridge::iterate( uint16_t _set, float *_current ) {
	int16_t intvalue;
	if ( _set > *_current ) {
		*_current += accIncrement;
	}
	else *_current -= accIncrement;
	if ( *_current < 0 ) {
		*_current = 0;
		intvalue = 0;
	}
	else intvalue = *_current;
	return intvalue;
}



} // namespace
