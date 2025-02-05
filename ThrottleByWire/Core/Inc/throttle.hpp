/*
 * throttle.hpp
 *
 *  Created on: Nov 14, 2024
 *      Author: rolan
 */


#ifndef THROTTLE_HPP_
#define THROTTLE_HPP_

#include <main.hpp>
#include "stm32f1xx_hal.h"

class Throttle {

	public:
		//Constructor
		Throttle();
		//Init function
		void initThrottle();
		//Read out function
		uint8_t readThrottle();

	private:


};


#endif /* INC_THROTTLE_HPP_ */
