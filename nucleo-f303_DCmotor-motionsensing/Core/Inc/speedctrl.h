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
	Speedcontrol(Setpoint *_set, Actual *_act, PID *_pid, PWM *_pwm);
	virtual ~Speedcontrol();
	void run();
	void stop();
	float getActualVelocity() { return actVelocity; };
	void stateMachine();
private:
	Setpoint *set;
	Actual *act;
	PID *pid;
	PWM *pwm;

	float refVelocity;
	float actVelocity;
	float avgVelocity;
	float actPosition;


	state setState;
	state controllerState;
};

} // namespace

#endif /* speedcontrol_H_ */
