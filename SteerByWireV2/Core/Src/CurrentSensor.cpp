/*
 * CurrentSensor.cpp
 *
 *  Created on: Jan 5, 2025
 *      Author: rolan
 */

#include "CurrentSensor.hpp"

// Constructor: Initializes the ADC instance for the current sensor
CurrentSensor::CurrentSensor(ADC_HandleTypeDef* hadc, float refVoltage, float sensitivity, float offset)
    : currentAdc(hadc), vRef(refVoltage), sensitivity(sensitivity), offset(offset) {}

void CurrentSensor::start(){
	HAL_ADC_Start(currentAdc);
}

float CurrentSensor::getCurrent(){
	HAL_ADC_PollForConversion(currentAdc, 10);
	float adcValue = HAL_ADC_GetValue(currentAdc);

	return (offset - (adcValue * (vRef/4095)))/sensitivity;
}

void CurrentSensor::stop(){
	HAL_ADC_Stop(currentAdc);
}
