/*
 * encoder.hpp
 *
 *  Created on: Nov 21, 2024
 *      Author: rolan
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stm32f1xx_hal.h"
#include "motorControl.hpp"
#include "CurrentSensor.hpp"

class Encoder{
	private:
		MotorControl* motorControl;
		CurrentSensor* currentSensor;
		TIM_HandleTypeDef* encoder1;
		TIM_HandleTypeDef* encoder2;
		uint16_t maxEncoderCount;

	public:
		Encoder(MotorControl* motorControl,CurrentSensor* currentSensor, TIM_HandleTypeDef* enc1, TIM_HandleTypeDef* enc2, uint16_t maxCount);
		uint32_t readEncoder(TIM_HandleTypeDef* enc);
		void resetEncoderCount(TIM_HandleTypeDef* enc);
		void calibrateEncoder(TIM_HandleTypeDef* enc, float stallCurrent);
		int16_t calculateAngle(uint16_t count);
		int16_t encoderCompare();
};


#endif /* INC_ENCODER_HPP_ */
