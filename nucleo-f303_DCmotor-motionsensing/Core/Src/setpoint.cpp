/*
 * Setpoint.cpp
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#include "setpoint.h"

namespace setpoint {

Setpoint::Setpoint() {
	// TODO Auto-generated constructor stub
	max = 1e6;
	target = 0.0;
	increment = 0.0;
	current = 0.0;
}

Setpoint::Setpoint( float _max, float _increment, bool _inRPM ) : Setpoint() {
	if ( _inRPM ) {
		max = RPM2PulsesPerSample( _max );
		increment = RPM2PulsesPerSample( _increment );
	}
	else {
		max = _max;
		increment = _increment;
	}
}

Setpoint::~Setpoint() {
	// TODO Auto-generated destructor stub
}

float Setpoint::iterate() {
	if ( target > current ) {
		current += increment;
		if ( current > target ) current = target;
	}
	else if ( target < current ) {
		current -= increment;
		if ( current < target ) current = target;
	}
	else current = target;
	return current;
}

} // namespace
