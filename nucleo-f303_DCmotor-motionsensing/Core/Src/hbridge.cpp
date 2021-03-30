/*
 * hbridge.cpp
 *
 *  Created on: 23.3.2021
 *      Author: martti
 */

#include "hbridge.h"
#include "main.h"

int32_t instantVelocity;
int32_t avgVelocity;
int32_t currentPosition;

namespace H_bridge {

hbridge::hbridge() {
	setState = state_stop;
	controllerState = state_stop;
	dir = bridge_idle;
	setSpeed = RPM2PulsesPerSample( 500.0 );
	accIncrement = RPM2PulsesPerSample( 100.0 );

	pid.Kp = 50.0;
	pid.Ki = 0.1;
	pid.Kd = 0;
	arm_pid_init_f32( &pid, true );

	htim = NULL;
}

void hbridge::begin(TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef* _in_A_port, uint16_t _in_A_pin, GPIO_TypeDef* _in_B_port, uint16_t _in_B_pin) {
	htim = _htim;
	channel = _channel;
	in_A_port = _in_A_port;
	in_A_pin = _in_A_pin;
	in_B_port = _in_B_port;
	in_B_pin = _in_B_pin;
}

hbridge::~hbridge() {
	// TODO Auto-generated destructor stub
}

// set shaft speed setpoint in RPM
void hbridge::setSpeedReference( float _setSpeed ) {

	setSpeed = RPM2PulsesPerSample( _setSpeed );
}


void hbridge::setAccReference(float _setAcc ) {
	accIncrement = RPM2PulsesPerSample( _setAcc );
}

void hbridge::setPGain( float _gain ) {
	pid.Kp = _gain;
	arm_pid_init_f32( &pid, false );
}

void hbridge::setIGain( float _gain ) {
	pid.Ki = _gain;
	arm_pid_init_f32( &pid, false );
}

void hbridge::setDGain(  float _gain ){
	pid.Kd = _gain;
	arm_pid_init_f32( &pid, false );
}

void hbridge::updateMotorSpeed() {
	static int32_t previousPosition = 0;
	static uint8_t vIndex = 0;
	static int32_t velocity[NUMVELOCITYSAMPLES];
	static int32_t vSum = 0;

	currentPosition = TIM2->CNT;
	vSum -= velocity[vIndex];
	instantVelocity = velocity[vIndex] = currentPosition - previousPosition;
	previousPosition = currentPosition;
	vSum += velocity[vIndex];
	vIndex++;
	vIndex &= ( NUMVELOCITYSAMPLES -1 );
	avgVelocity = vSum / NUMVELOCITYSAMPLES;
}

void hbridge::stateMachine() {
	float fD;
	uint16_t D;
	switch ( setState ) {
		case state_stop: {
			switch ( controllerState ) {
				case state_stop: {
					// ei tehdä mitään
					break;
				}
				case state_run: {
					updateMotorSpeed();
					setPointIterate( 0.0 );
					if ( commandSpeed < EPSILON && commandSpeed > -EPSILON ) {
						HAL_TIM_PWM_Stop(htim, channel);
						HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
						controllerState = state_stop;
					}
					break;
				}
			}
			break;
		}
		case state_run: {
			// ajotilassa päivitetään position ja nopeuden oloarvot
			updateMotorSpeed();
			// suoritetaan ohjauksen tilakone
			switch ( controllerState ) {
				case state_stop: {
					arm_pid_reset_f32( &pid );
					if ( setSpeed > 0 ) {
						HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
						dir = bridge_forward;
					}
					else {
						HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_SET);
						dir = bridge_reverse;
					}
					HAL_TIM_PWM_Start(htim, channel);
					controllerState = state_run;
					break;
				}
				case state_run: {
					// iteroidaan nopeuden ohjearvo
					setPointIterate(setSpeed);
					// asetetaan sillan suunta
					if ( commandSpeed > EPSILON ) {
						if ( dir != bridge_forward ) {
							HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_SET);
							dir = bridge_forward;
						}
					}
					else if ( commandSpeed < -EPSILON ) {
						if ( dir != bridge_reverse ) {
							HAL_GPIO_WritePin(IN1_A_GPIO_Port, IN1_A_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(IN1_B_GPIO_Port, IN1_B_Pin, GPIO_PIN_SET);
							dir = bridge_reverse;
						}
					}
					break;
				}
			}
			break;
		}
	}
	if ( controllerState == state_run )	{// säädetään pwm
		fD = arm_pid_f32( &pid, commandSpeed - instantVelocity );
		if ( fD < 0 ) D = -fD;
		else D = fD;
		if ( D > 1000 ) D = 1000;
		htim->Instance->CCR3 = D;
	}
}

void hbridge::setPointIterate(float targetSpeed) {

	if ( targetSpeed > ( commandSpeed + EPSILON )) {
		commandSpeed += accIncrement;
		if ( commandSpeed > targetSpeed ) commandSpeed = targetSpeed;
	}
	else if ( targetSpeed < ( commandSpeed - EPSILON )) {
		commandSpeed -= accIncrement;
		if ( commandSpeed < targetSpeed ) commandSpeed = targetSpeed;
	}
	else commandSpeed = targetSpeed;
}

hbridge bridge1;

} // namespace



void runStateMachine() { H_bridge::bridge1.stateMachine(); }

