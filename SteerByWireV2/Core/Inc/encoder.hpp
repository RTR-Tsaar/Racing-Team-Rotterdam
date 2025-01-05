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

class Encoder{
	private:
		MotorControl* motorControl;
		TIM_HandleTypeDef* encoder1;
		TIM_HandleTypeDef* encoder2;
		uint16_t maxEncoderCount;
		float countsPerRev;

	public:
		Encoder(MotorControl* motorControl, TIM_HandleTypeDef* enc1, TIM_HandleTypeDef* enc2, uint16_t maxCount, float countsPerRevolution);
		uint32_t readEncoder(TIM_HandleTypeDef* enc);
		void resetEncoderCount(TIM_HandleTypeDef* enc);
		void calibrateEncoder(TIM_HandleTypeDef* enc, uint16_t motorCurrent);
		int16_t calculateAngle(uint16_t count);
		int16_t encoderCompare();
};


#endif /* INC_ENCODER_HPP_ */
