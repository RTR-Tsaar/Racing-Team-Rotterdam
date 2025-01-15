/*
 * CurrentSensor.hpp
 *
 *  Created on: Jan 5, 2025
 *      Author: rolan
 */

#ifndef CURRENT_SENSOR_HPP
#define CURRENT_SENSOR_HPP

#include "stm32f1xx_hal.h"

class CurrentSensor {
private:
    ADC_HandleTypeDef* currentAdc; // ADC handle for the sensor
    float vRef;                   // Reference voltage
    float sensitivity;            // Sensitivity of the sensor (mV/A)
    float offset;                 // Offset voltage when no current is flowing

    // Reads the raw ADC value
    uint16_t readADC();

    // Converts the ADC value to voltage
    float adcToVoltage(uint16_t adcValue);

public:
    // Constructor to initialize the sensor with ADC handle and sensor parameters
    CurrentSensor(ADC_HandleTypeDef* hadc, float refVoltage, float sensitivity, float offset);

    void start();
    void stop();
    // Gets the current from the sensor
    float getCurrent();
};

#endif // CURRENT_SENSOR_HPP

