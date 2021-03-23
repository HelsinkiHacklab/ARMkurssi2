/*
 * hbridge.h
 *
 *  Created on: 23.3.2021
 *      Author: Kremmen
 */

#ifndef SRC_HBRIDGE_H_
#define SRC_HBRIDGE_H_

#include "GPIO.h"
#include "tim.h"

namespace H_bridge {

enum state {state_stop, state_cutoff, state_brake, state_forward, state_reverse };

class hbridge {
public:
	hbridge(TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef* _in_A_port, uint16_t _in_A_pin, GPIO_TypeDef* _in_B_port, uint16_t _in_B_pin);
	virtual ~hbridge();
	void setSpeedReference( uint16_t _setSpeed );
	void setAccReference(float _setAcc );
	inline void forward() 	{ commandState = state_forward; };
	inline void reverse() 	{ commandState = state_reverse; };
	inline void stop() 		{ commandState = state_stop; };
	inline void cutoff() 	{ commandState = state_cutoff; };
	inline void brake() 	{ commandState = state_brake; };
	void stateMachine();
private:
	uint16_t iterate( uint16_t _set, float *_current);
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef* in_A_port;
	uint16_t in_A_pin;
	GPIO_TypeDef* in_B_port;
	uint16_t in_B_pin;

	state commandState;
	state actualState;

	uint16_t setSpeed;
	float currentSpeed;
	uint16_t pwmSpeed;
	float accIncrement;

	uint16_t delayCounter;

};

} // namespace

#endif /* SRC_HBRIDGE_H_ */
