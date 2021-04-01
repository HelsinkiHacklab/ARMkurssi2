/*
 * Speedcontrol.h
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#ifndef speedcontrol_H_
#define speedcontrol_H_

#include "setpoint.h"
#include "actual.h"
#include "pid.h"
#include "pwm.h"

namespace speedcontrol {

using namespace setpoint;
using namespace actual;
using namespace pid;
using namespace pwm;

enum state {state_stop, state_run };

class Speedcontrol {
public:
	Speedcontrol(TIM_HandleTypeDef *_htim, Setpoint *_set, Actual *_act, PID *_pid, PWM *_pwm);
	virtual ~Speedcontrol();
	bool begin();
	void setTargetRPM( float _RPM ) { setVelocity = _RPM; set->setTargetRPM( setVelocity );};
	void setIncrementRPM( float _acc ) {set->setIncrementRPM( _acc ); };
	void run() { set->setTargetRPM( setVelocity ); setState = state_run; };
	void stop() {  set->setTargetRPM( 0.0 ); setState = state_stop; };
	float getActualVelocity() { return actVelocity; };
	void stateMachine();
private:
	TIM_HandleTypeDef *htim;
	Setpoint *set;
	Actual *act;
	PID *pid;
	PWM *pwm;

	float setVelocity;
	float refVelocity;
	float actVelocity;
	float avgVelocity;
	float actPosition;


	state setState;
	state controllerState;
};

} // namespace

#endif /* speedcontrol_H_ */
