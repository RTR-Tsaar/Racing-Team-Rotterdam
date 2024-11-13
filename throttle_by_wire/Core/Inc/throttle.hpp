/*
 * throttle.hpp
 *
 *  Created on: Nov 12, 2024
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
		void initADC();
		//Read out function
		int readThrottle();

	private:


};



#endif /* THROTTLE_HPP_ */
