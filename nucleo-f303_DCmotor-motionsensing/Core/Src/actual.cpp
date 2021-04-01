/*
 * actual.cpp
 *
 *  Created on: 31.3.2021
 *      Author: Kremmen
 */

#include "actual.h"

namespace actual {

Actual::Actual(TIM_HandleTypeDef *_htim, uint32_t _channel) {
	htim = _htim;
	channel = _channel;
	previousPosition = 0;
}

Actual::~Actual() {
}

AvgActual::AvgActual(TIM_HandleTypeDef *_htim, uint32_t _channel) :
	Actual( _htim, _channel ) {
	vIndex = 0;
	vSum = 0;
}

AvgActual::~AvgActual() {
}


float Actual::measureActual() {
	currentPosition = htim->Instance->CNT;
	currentVelocity = currentPosition - previousPosition;
	previousPosition = currentPosition;
	return currentVelocity;
}

float AvgActual::measureActual() {
	int32_t tmp;
	vSum -= velocity[vIndex];
	velocity[vIndex] = tmp = Actual::measureActual();
	vSum += velocity[vIndex];
	vIndex++;
	vIndex &= ( NUMVELOCITYSAMPLES -1 );
	return tmp;
}

} // namespace
