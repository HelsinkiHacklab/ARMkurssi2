/*
 * pid.h
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

namespace pid {

class PID {
public:
	PID(float _maxOutput, bool _antiWindup, float _Kp, float _Ki, float _Kd );
	virtual ~PID();
	virtual bool begin() { init( true); return true; };
	void setKp( float _Kp ) { Kp = _Kp; init(false); };
	void setKi( float _Ki ) { Ki = _Ki; init(false); };
	void setKd( float _Kd ) { Kd = _Kd; init(false); };
	void reset();
	float run( float _set, float _actual );
private:
	void init(bool _reset);
	float maxOutput;
	bool antiWindup;
	float Kp;
	float Ki;
	float Kd;
	float A0;
	float A1;
	float A2;
	float state[3];
};

} // namespace

#endif /* SRC_PID_H_ */
