/*
 * positioning.h
 *
 *  Created on: Apr 27, 2021
 *      Author: martti
 */

#ifndef SRC_POSITIONING_H_
#define SRC_POSITIONING_H_

#include "main.h"


namespace Positioning {

enum class direction : int8_t { reverse=-1, stop=0, forward=1 };
enum trajectoryForm { triangular, trapezoidal };


class moveGenerator {
public:
	moveGenerator( float _maxVelocity, float _maxAcceleration );
	virtual ~moveGenerator();
	void begin();
	float update();
	bool isInPosition() { return inPosition; };
	float position() { return currPosition; };
	float velocity() { return currVelocity; };
	float acceleration() { return currAcceleration; };
	void setTargetPosition( float _targetPosition );
	void setMaxVelocity( float _maxVelocity ) { maxVelocity = _maxVelocity; };
	void setMaxAcceleration( float _maxAcceleration ) { maxAcceleration = _maxAcceleration; };
private:
	void calcMotionProfile( float positionReference );
	direction getDirection( float xVal );
	float targetPosition;
	float maxVelocity;
	float maxAcceleration;

	// current motion values (Z0)
	float currPosition;
	float currVelocity;
	float currAcceleration;

	// previous iteration motion values (Z-1)
	float prevPosition;
	float prevStartPosition;
	float prevVelocity;
	float tmpAvgVel;

	// distances to key trajectory points
	float distancetoBrake;
	float distancetoAccelerate;
	float distancetoVelocity;
	float distancetoDecelerate;
	float distanceTotal;

	// times to key trajectory points
	float timetoBrake;
	float timetoAccelerate;
	float timetoVelocity;
	float timetoDecelerate;

	// iteration time counters
	uint32_t prevTimeStamp;
	uint32_t lastTimeStamp;
	uint32_t elapsedTime;

	// trajectory envelope info
	direction dir;
	trajectoryForm form;

	bool inPosition;
};

} // namespace

#endif /* SRC_POSITIONING_H_ */
