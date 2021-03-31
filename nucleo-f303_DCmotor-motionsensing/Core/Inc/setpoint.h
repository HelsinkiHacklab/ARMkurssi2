/*
 * Setpoint.h
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#ifndef setpoint_H_
#define setpoint_H_

namespace setpoint {

class Setpoint {
public:
	Setpoint();
	Setpoint( float _max, float _increment );
	virtual ~Setpoint();
	void reset() { current = 0.0; };
	void setMaximum( float _max ) { max = _max; };
	void setTarget( float _target ) { target = _target; };
	float getTarget() { return target; };
	void setIncrement( float _increment ) { increment = _increment; };
	float iterate();
private:
	float max;
	float target;
	float increment;
	float current;
};


} // namespace

#endif /* setpoint_H_ */
