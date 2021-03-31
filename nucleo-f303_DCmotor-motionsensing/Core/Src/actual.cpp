/*
 * actual.cpp
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#include "actual.h"

namespace actual {

Actual::Actual(TIM_HandleTypeDef *_htim, uint32_t _channel) {
	htim = _htim;
	channel = _channel;
	previousPosition = 0;
}

Actual::~Actual() {
	// TODO Auto-generated destructor stub
}

AvgActual::AvgActual(TIM_HandleTypeDef *_htim, uint32_t _channel) :
		Actual::Actual( _htim, _channel ) {
	vIndex = 0;
	vSum = 0;

}

float Actual::getActual() {
	float tmp;
	currentPosition = TIM2->CNT;
	tmp = currentPosition - previousPosition;
	previousPosition = currentPosition;
	return tmp;
}

float AvgActual::getActual() {
	vSum -= velocity[vIndex];
	velocity[vIndex] = Actual::getActual();
	vSum += velocity[vIndex];
	vIndex++;
	vIndex &= ( NUMVELOCITYSAMPLES -1 );
	return vSum / NUMVELOCITYSAMPLES;

}

} // namespace
