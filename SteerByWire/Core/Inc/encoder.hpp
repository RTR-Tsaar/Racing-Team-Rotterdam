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

	public:
		Encoder();
		int readEncoder();
};


#endif /* INC_ENCODER_HPP_ */
