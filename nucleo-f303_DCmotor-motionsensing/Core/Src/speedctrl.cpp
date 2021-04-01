/*
 * Speedcontrol.cpp
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#include <cmath>
#include "speedctrl.h"

namespace speedcontrol {

Speedcontrol::Speedcontrol(Setpoint *_set, Actual *_act, PID *_pid, PWM *_pwm) {
	set = _set;
	act = _act;
	pid = _pid;
	pwm = _pwm;
}

Speedcontrol::~Speedcontrol() {
	// TODO Auto-generated destructor stub
}

void Speedcontrol::stateMachine() {
	switch ( setState ) {
		case state_stop: {
			switch ( controllerState ) {
				case state_stop: {
					// ei tehdä mitään
					break;
				}
				case state_run: {
					refVelocity = set->iterate();
					actVelocity = act->measureActual();
					if ( refVelocity < EPSILON && refVelocity > -EPSILON ) {
						pwm->stop();
						controllerState = state_stop;
					}
					break;
				}
			}
			break;
		}
		case state_run: {
			// ajotilassa päivitetään position ja nopeuden oloarvot
			actVelocity = act->measureActual();
			// suoritetaan ohjauksen tilakone
			switch ( controllerState ) {
				case state_stop: {
					pid->reset();
					if ( set->getTarget() > 0.0 ) pwm->forward();
					else pwm->reverse();
					controllerState = state_run;
					break;
				}
				case state_run: {
					// iteroidaan nopeuden ohjearvo
					refVelocity = set->iterate();
					// asetetaan sillan suunta
					if ( refVelocity > EPSILON ) {
						if ( pwm->state() != bridge_forward ) {
							pwm->forward();
						}
					}
					else if ( refVelocity < -EPSILON ) {
						if ( pwm->state() != bridge_reverse ) {
							pwm->reverse();
						}
					}
					break;
				}
			}
			break;
		}
	}
	if ( controllerState == state_run )	{// säädetään pwm
		pwm->D( abs( pid->run( refVelocity, actVelocity ) ) );
	}
}


} //namespace
