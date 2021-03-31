/*
 * pid.cpp
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */
#include "string.h"
#include "pid.h"

namespace pid {

PID::PID(float _maxOutput, bool _antiWindup, float _Kp, float _Ki, float _Kd ) {

	maxOutput = _maxOutput;
	antiWindup = _antiWindup;
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	init(true);
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

void PID::reset() {
	memset(state, 0, 3U * sizeof(float));
}

float PID::run( float _set, float _actual ) {
	float error;
	float out;

	error = _set - _actual;
	/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    out = (A0 * error) +  (A1 * state[0]) + (A2 * state[1]) + state[2];
    if ( antiWindup ) {
    	if ( out > maxOutput ) out = maxOutput;
    	else if ( out < -maxOutput ) out = -maxOutput;
    }
    /* Update state */
    state[1] = state[0];
    state[0] = error;
    state[2] = out;

    /* return to application */
    return (out);
}

void PID::init(bool _reset) {
	A0 = Kp + Ki + Kd;
	A1 = (-Kp) - ((float) 2.0f * Kd);
	A2 = Kd;
	if (_reset) reset();
}

} // namespace
