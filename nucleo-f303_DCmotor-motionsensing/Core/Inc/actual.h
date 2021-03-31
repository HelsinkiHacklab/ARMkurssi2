/*
 * actual.h
 *
 *  Created on: 31.3.2021
 *      Author: martti
 */

#ifndef SRC_ACTUAL_H_
#define SRC_ACTUAL_H_

#include "tim.h"

namespace actual {

class Actual {
public:
	Actual(TIM_HandleTypeDef *_htim, uint32_t _channel);
	virtual ~Actual();
	float getActual();
private:
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	int32_t previousPosition;
};


class AvgActual: public Actual {
	AvgActual(TIM_HandleTypeDef *_htim, uint32_t _channel);
	virtual ~AvgActual();
	float getActual();
private:
	uint8_t vIndex;
	int32_t velocity[NUMVELOCITYSAMPLES];
	int32_t vSum;
};

} // namespace

#endif /* SRC_ACTUAL_H_ */
