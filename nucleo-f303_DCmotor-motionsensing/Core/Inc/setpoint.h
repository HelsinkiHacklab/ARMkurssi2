/*
 * Setpoint.h
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#ifndef setpoint_H_
#define setpoint_H_

#include "main.h"

namespace setpoint {

class Setpoint {
public:
	Setpoint();
	Setpoint( float _max, float _increment, bool _inRPM );
	virtual ~Setpoint();
	virtual bool begin() { return true; };
	void reset() { current = 0.0; };
	void setMaximum( float _max ) { max = _max; };
	void setTarget( float _target ) { target = _target; };
	void setTargetRPM( float _target ) { target = RPM2PulsesPerSample( _target ); };
	float getTarget() { return target; };
	float getTargetRPM() { return PulsesPerSample2RPM( target ); };
	void setIncrement( float _increment ) { increment = _increment; };
	void setIncrementRPM( float _increment ) { increment = RPM2PulsesPerSample( _increment ); };
	float iterate();
private:
	inline float RPM2PulsesPerSample( float _RPM) { return _RPM / (60*SAMPLING_FREQ) * ENC_PPR; };
	inline float PulsesPerSample2RPM( float _samples) { return _samples * (60*SAMPLING_FREQ) / ENC_PPR; };
	float max;
	float target;
	float increment;
	float current;
};


} // namespace

#endif /* setpoint_H_ */
