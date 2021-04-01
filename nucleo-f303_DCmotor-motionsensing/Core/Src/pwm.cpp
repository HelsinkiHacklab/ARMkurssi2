/*
 * pwm.cpp
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#include "pwm.h"

namespace pwm {

PWM::PWM(TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef* _in_A_port, uint16_t _in_A_pin, GPIO_TypeDef* _in_B_port, uint16_t _in_B_pin) {
	// TODO Auto-generated constructor stub
	htim = _htim;
	channel = _channel;
	in_A_port = _in_A_port;
	in_A_pin = _in_A_pin;
	in_B_port = _in_B_port;
	in_B_pin = _in_B_pin;

	stat = bridge_idle;
}

PWM::~PWM() {
	// TODO Auto-generated destructor stub
}
void PWM::forward() {
	HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(htim, channel);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	stat = bridge_forward;
}

void PWM::reverse() {
	HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(htim, channel);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	stat = bridge_reverse;
}

void PWM::stop() {
	HAL_TIM_PWM_Stop(htim, channel);
	HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	stat = bridge_idle;
	}

void PWM::D(float _d) {
	htim->Instance->CCR3 = _d; // CCR3 pitäisi oikeasti johtaa muutujasta channel
}

} //namespace
