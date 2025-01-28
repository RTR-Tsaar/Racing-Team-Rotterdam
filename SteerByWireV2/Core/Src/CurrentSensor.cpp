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

//The start method initializes the adc channel used for measuring current
void CurrentSensor::start(){
	HAL_ADC_Start(currentAdc);
}

//This method is used to read out the adc channel and to convert the raw data received into usable Amps
float CurrentSensor::getCurrent(){
	HAL_ADC_PollForConversion(currentAdc, 10);
	float adcValue = HAL_ADC_GetValue(currentAdc);

	//Depending on how the current sensor is wired the offset may have to be inverted.
	return (offset - (adcValue * (vRef/4095)))/sensitivity;
}

//This method turns of the adc channel when we are done with it.
void CurrentSensor::stop(){
	HAL_ADC_Stop(currentAdc);
}
