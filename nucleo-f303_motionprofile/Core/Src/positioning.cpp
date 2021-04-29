/*
 * positioning.cpp
 *
 *  Created on: Apr 27, 2021
 *      Author: martti
 */

#include <math.h>
#include "positioning.h"

namespace Positioning {


#define epsilon 0.000001F



moveGenerator::moveGenerator(float _maxVelocity, float _maxAcceleration ) :
	maxVelocity(_maxVelocity), maxAcceleration(_maxAcceleration) {
	begin();
}

moveGenerator::~moveGenerator() {
	// TODO Auto-generated destructor stub
}

void moveGenerator::begin() {
	prevTimeStamp = HAL_GetTick();
	lastTimeStamp = prevTimeStamp;
	elapsedTime = 0;

	currPosition = 0;
	prevPosition = 0;
	prevStartPosition = 0;
	currVelocity = 0;
	currAcceleration = 0;
	prevVelocity = 0;
	tmpAvgVel = 0;
	distancetoBrake = 0;
	distancetoAccelerate = 0;
	distancetoVelocity = 0;
	distancetoDecelerate = 0;
	distanceTotal = 0;
	timetoBrake = 0;
	timetoAccelerate = 0;
	timetoVelocity = 0;
	timetoDecelerate = 0;

	dir = direction::stop;
	form = trapezoidal;
	inPosition = true;
}

void moveGenerator::setTargetPosition( float _targetPosition ) {
	targetPosition = _targetPosition;
	if ( targetPosition != currPosition ) {
		inPosition = false;
	}
}

float moveGenerator::update() {
	if (prevStartPosition != targetPosition) {
		inPosition = false;
		// Shift state variables
		prevStartPosition = targetPosition;
		prevPosition = currPosition;
		prevVelocity = currVelocity;
		prevTimeStamp = lastTimeStamp;

		// Calculate braking time and distance (in case is neeeded)
		timetoBrake = abs(prevVelocity) / maxAcceleration;
		distancetoBrake = timetoBrake * abs(prevVelocity) / 2;

		// Caculate Sign of motion
		dir = getDirection(targetPosition - (prevPosition + (int8_t)getDirection(prevVelocity) * distancetoBrake));

		if (dir != getDirection(prevVelocity)) { // means brake is needed
			timetoAccelerate = (maxVelocity / maxAcceleration);
			distancetoAccelerate = timetoAccelerate * (maxVelocity / 2);
		}
		else {
			timetoBrake = 0;
			distancetoBrake = 0;
			timetoAccelerate = (maxVelocity - abs(prevVelocity)) / maxAcceleration;
			distancetoAccelerate = timetoAccelerate * (maxVelocity + abs(prevVelocity)) / 2;
		}

		// Calculate total distance to go after braking
		distanceTotal = abs(targetPosition - prevPosition + (int8_t)dir * distancetoBrake);

		timetoDecelerate = maxVelocity / maxAcceleration;
		distancetoDecelerate = timetoDecelerate * (maxVelocity) / 2;
		distancetoVelocity = distanceTotal - (distancetoAccelerate + distancetoDecelerate);
		timetoVelocity = distancetoVelocity / maxVelocity;

		if (timetoVelocity > 0) {
			form = trapezoidal;
		}
		else {
			form = triangular;
			// Recalculate distances and periods
			if (dir != getDirection(prevVelocity)) { // means brake is needed
				tmpAvgVel = sqrt(maxAcceleration*(distanceTotal));
				timetoAccelerate = (tmpAvgVel / maxAcceleration);
				distancetoAccelerate = timetoAccelerate * (tmpAvgVel / 2);
			}
			else {
				timetoBrake = 0;
				distancetoBrake = 0;
				distanceTotal = abs(targetPosition - prevPosition);      // recalculate total distance
				tmpAvgVel = sqrt(0.5*prevVelocity*prevVelocity + maxAcceleration*distanceTotal);
				timetoAccelerate = (tmpAvgVel - abs(prevVelocity)) / maxAcceleration;
				distancetoAccelerate = timetoAccelerate * (tmpAvgVel + abs(prevVelocity)) / 2;
			}
			timetoDecelerate = tmpAvgVel / maxAcceleration;
			distancetoDecelerate = timetoDecelerate * (tmpAvgVel) / 2;
		}

	}

	unsigned long time = HAL_GetTick();
	// Calculate time since last set-point change
	elapsedTime = (time - prevTimeStamp);
	// Calculate new setpoint
	calcMotionProfile(targetPosition);
	// Update last time
	lastTimeStamp = time;

	//calculateTrapezoidalProfile(setpoint);
	return currPosition;

}

void moveGenerator::calcMotionProfile( float positionReference ) {
	float t = (float)elapsedTime;
	t /= 1000.0;

	if (form == trapezoidal) {
		if (t <= (timetoBrake+timetoAccelerate)) {
			currPosition = prevPosition + prevVelocity*t + (int8_t)dir * 0.5*maxAcceleration*t*t;
			currVelocity = prevVelocity + (int8_t)dir * maxAcceleration*t;
			currAcceleration = (int8_t)dir * maxAcceleration;
		}
		else if (t > (timetoBrake+timetoAccelerate) && t < (timetoBrake+timetoAccelerate+timetoVelocity)) {
			currPosition = prevPosition + (int8_t)dir * (-distancetoBrake + distancetoAccelerate + maxVelocity*(t-timetoBrake-timetoAccelerate));
			currVelocity = (int8_t)dir * maxVelocity;
			currAcceleration = 0;
		}
		else if (t >= (timetoBrake+timetoAccelerate+timetoVelocity) && t < (timetoBrake+timetoAccelerate+timetoVelocity+timetoDecelerate)) {
			currPosition = prevPosition + (int8_t)dir * (-distancetoBrake + distancetoAccelerate + distancetoVelocity + maxVelocity*(t-timetoBrake-timetoAccelerate-timetoVelocity) - 0.5*maxAcceleration*(t-timetoBrake-timetoAccelerate-timetoVelocity)*(t-timetoBrake-timetoAccelerate-timetoVelocity));
			currVelocity = (int8_t)dir * (maxVelocity - maxAcceleration*(t-timetoBrake-timetoAccelerate-timetoVelocity));
			currAcceleration = - (int8_t)dir * maxAcceleration;
		}
		else {
			currPosition = positionReference;
			currVelocity = 0;
			currAcceleration = 0;
			inPosition = true;
			dir = direction::stop;
		}
	}
	else {
		if (t <= (timetoBrake+timetoAccelerate)) {
			currPosition = prevPosition + prevVelocity*t + (int8_t)dir * 0.5*maxAcceleration*t*t;
			currVelocity = prevVelocity + (int8_t)dir * maxAcceleration*t;
			currAcceleration = (int8_t)dir * maxAcceleration;
		}
		else if (t > (timetoBrake+timetoAccelerate) && t < (timetoBrake+timetoAccelerate+timetoDecelerate)) {
			currPosition = prevPosition + (int8_t)dir * (-distancetoBrake + distancetoAccelerate + tmpAvgVel*(t-timetoBrake-timetoAccelerate) - 0.5*maxAcceleration*(t-timetoBrake-timetoAccelerate)*(t-timetoBrake-timetoAccelerate));
			currVelocity = (int8_t)dir * (tmpAvgVel - maxAcceleration*(t-timetoBrake-timetoAccelerate));
			currAcceleration = - (int8_t)dir * maxAcceleration;
		}
		else {
			currPosition = positionReference;
			currVelocity = 0;
			currAcceleration = 0;
			inPosition = true;
			dir = direction::stop;
		}
	}
}


direction moveGenerator::getDirection( float xVal ) {
	if ( xVal > epsilon ) return direction::forward;
	if ( xVal < -epsilon ) return direction::reverse;
	return direction::stop;
}

} // namespace
