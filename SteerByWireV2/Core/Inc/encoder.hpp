/*
 * encoder.hpp
 *
 *  Created on: Nov 21, 2024
 *      Author: rolan
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stm32f1xx_hal.h"

class Encoder{
	private:
		TIM_HandleTypeDef* encoder1;
		TIM_HandleTypeDef* encoder2;
		uint32_t maxEncoderCount;
		float countsPerRev;
		uint32_t readEncoder(TIM_HandleTypeDef* enc);
		float calculateAngle(uint32_t count);

	public:
		Encoder(TIM_HandleTypeDef* enc1, TIM_HandleTypeDef* enc2, uint32_t maxCount, float countsPerRevolution);
		float encoderCompare();
};


#endif /* INC_ENCODER_HPP_ */
