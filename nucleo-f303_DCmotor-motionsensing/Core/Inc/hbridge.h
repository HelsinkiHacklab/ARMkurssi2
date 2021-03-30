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
#include "dsp/controller_functions.h"


namespace H_bridge {

enum state {state_stop, state_run };
enum bridgeDirection {bridge_idle, bridge_forward, bridge_reverse };

class hbridge {
public:
	hbridge();
	virtual ~hbridge();
	void begin(TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef* _in_A_port, uint16_t _in_A_pin, GPIO_TypeDef* _in_B_port, uint16_t _in_B_pin);
	void setSpeedReference( float _setSpeed );
	void setAccReference(float _setAcc );
	void setPGain( float _gain );
	void setIGain( float _gain );
	void setDGain( float _gain );
	void run() { setState = state_run; };
	void stop() { setState = state_stop; };
	void stateMachine();
private:
	inline float RPM2PulsesPerSample( float _RPM) { return _RPM/(60*SAMPLING_FREQ)*ENC_PPR; };
	void setPointIterate(float targetSpeed);
	void updateMotorSpeed();
	arm_pid_instance_f32 pid;
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef* in_A_port;
	uint16_t in_A_pin;
	GPIO_TypeDef* in_B_port;
	uint16_t in_B_pin;

	state setState;
	state controllerState;
	bridgeDirection dir;

	float setSpeed;
	float commandSpeed;
	float accIncrement;

};

extern hbridge bridge1;

} // namespace

#endif /* SRC_HBRIDGE_H_ */
