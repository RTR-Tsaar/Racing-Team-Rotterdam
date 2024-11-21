/*
 * throttle.cpp
 *
 *  Created on: Nov 14, 2024
 *      Author: rolan
 */


#include "throttle.hpp"

extern ADC_HandleTypeDef hadc1;  // This is the ADC handle, assuming ADC1 is used

Throttle::Throttle() {
    // Constructor implementation if needed
}

// Function to initialize ADC if it's not done in main
void Throttle::initThrottle() {

}

// Function to read the throttle value from an analog pin
uint8_t Throttle::readThrottle() {
	HAL_ADC_PollForConversion(&hadc1,8);
	uint16_t readValue = HAL_ADC_GetValue(&hadc1);
	uint8_t percentage = (readValue*100)/4095;

	return percentage;
}


