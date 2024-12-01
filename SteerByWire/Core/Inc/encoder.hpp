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
		uint32_t readEncoder();
		uint32_t encoderCompare();
		uint8_t getCurrentAngle();

	public:
		Encoder();
};


#endif /* INC_ENCODER_HPP_ */
