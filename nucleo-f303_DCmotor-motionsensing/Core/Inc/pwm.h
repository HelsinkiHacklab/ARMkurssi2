/*
 * pwm.h
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#ifndef SRC_PWM_H_
#define SRC_PWM_H_

#include "tim.h"

namespace pwm {

enum bridgeState {bridge_idle, bridge_forward, bridge_reverse };

class PWM {
public:
	PWM(TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef* _in_A_port, uint16_t _in_A_pin, GPIO_TypeDef* _in_B_port, uint16_t _in_B_pin);
	virtual ~PWM();
	virtual bool begin() { return true; };
	void forward();
	void reverse();
	void stop();
	void D(float _d);
	bridgeState state() { return stat; };
private:
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef* in_A_port;
	uint16_t in_A_pin;
	GPIO_TypeDef* in_B_port;
	uint16_t in_B_pin;

	bridgeState stat;
};

} //namespace

#endif /* SRC_PWM_H_ */
