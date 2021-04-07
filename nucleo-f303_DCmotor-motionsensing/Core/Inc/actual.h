/*
 * actual.h
 *
 *  Created on: 31.3.2021
 *      Author: Kremmen
 */

#ifndef SRC_ACTUAL_H_
#define SRC_ACTUAL_H_

#include "tim.h"

namespace actual {

class Actual {
public:
	Actual(TIM_HandleTypeDef *_htim, uint32_t _channel);
	virtual ~Actual();
	virtual bool begin();
	virtual float measureActual();
	float getSpeed() { return currentVelocity; };
	float getSpeedRPM() { return PulsesPerSample2RPM( currentVelocity ); };
	int32_t getPosition() { return currentPosition; };
protected:
	inline float RPM2PulsesPerSample( float _RPM) { return _RPM / (60*SAMPLING_FREQ) * ENC_PPR; };
	inline float PulsesPerSample2RPM( float _samples) { return _samples * (60*SAMPLING_FREQ) / ENC_PPR; };
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	int32_t currentPosition;
	int32_t previousPosition;
	int32_t currentVelocity;
};


class AvgActual: public Actual {
public:
	AvgActual(TIM_HandleTypeDef *_htim, uint32_t _channel);
	virtual ~AvgActual();
	float measureActual();
	float getAvgSpeed() { return vSum / NUMVELOCITYSAMPLES; };
	float getAvgSpeedRPM() { return PulsesPerSample2RPM( vSum / NUMVELOCITYSAMPLES ); };
private:
	uint8_t vIndex;
	int32_t velocity[NUMVELOCITYSAMPLES];
	int32_t vSum;
};

} // namespace

#endif /* SRC_ACTUAL_H_ */
