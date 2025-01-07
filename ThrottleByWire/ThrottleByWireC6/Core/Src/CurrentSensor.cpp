/*
 * CurrentSensor.cpp
 *
 *  Created on: Jan 5, 2025
 *      Author: rolan
 */

#include "CurrentSensor.hpp"

// Constructor: Initializes the ADC instance for the current sensor
CurrentSensor::CurrentSensor(ADC_HandleTypeDef* hadc, float refVoltage, float sensitivity, float offset)
    : hadc(hadc), vRef(refVoltage), sensitivity(sensitivity), offset(offset) {}

float CurrentSensor::getCurrent(){
	HAL_ADC_PollForConversion(hadc, 10);
	float adcValue = HAL_ADC_GetValue(hadc);

	return (offset - (adcValue * (vRef/4095)))/sensitivity;
}

