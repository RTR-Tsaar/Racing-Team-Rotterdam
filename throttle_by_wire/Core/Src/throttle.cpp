/*
 * throttle.cpp
 *
 *  Created on: Nov 12, 2024
 *      Author: rolan
 */

#include "throttle.hpp"

extern ADC_HandleTypeDef hadc1;  // This is the ADC handle, assuming ADC1 is used

Throttle::Throttle() {
    // Constructor implementation if needed
}

// Function to initialize ADC if it's not done in main
void Throttle::initADC() {
    // ADC initialization code here if necessary
}

// Function to read the throttle value from an analog pin
int Throttle::readThrottle() {
    HAL_ADC_Start(&hadc1);  // Start ADC conversion

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {  // Wait for conversion
        int throttleValue = HAL_ADC_GetValue(&hadc1);  // Get ADC value
        HAL_ADC_Stop(&hadc1);  // Stop ADC after reading
        return throttleValue;  // Return the ADC value
    }

    HAL_ADC_Stop(&hadc1);  // Stop ADC if conversion fails
    return -1;  // Indicate an error if ADC read fails
}
