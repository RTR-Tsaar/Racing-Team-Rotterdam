/*
 * encoder.cpp
 *
 *  Created on: Nov 21, 2024
 *      Author: rolan
 */
 
#include "encoder.hpp"
#include "cmath"

Encoder::Encoder(TIM_HandleTypeDef* enc1, TIM_HandleTypeDef* enc2, uint32_t maxCount, float countsPerRevolution)
    : encoder1(enc1), encoder2(enc2), maxEncoderCount(maxCount), countsPerRev(countsPerRevolution) {
    // Constructor
};

uint32_t Encoder::readEncoder(TIM_HandleTypeDef* enc) {
    // Reads encoder data
    return __HAL_TIM_GET_COUNTER(enc);
}

float Encoder::calculateAngle(uint32_t count) {
    // Converts encoder count to angle
    return (static_cast<float>(count) / countsPerRev) * 360.0f;
}

float Encoder::encoderCompare() {
    uint32_t encoder1_count = readEncoder(encoder1);
    uint32_t encoder2_count = readEncoder(encoder2);
    int32_t deficit = static_cast<int32_t>(encoder1_count) - static_cast<int32_t>(encoder2_count);

    if (abs(deficit) > 5) {
        // Return NaN or an error flag if counts are inconsistent
        return -1.0f;
    }

    uint32_t averageCount = (encoder1_count + encoder2_count) / 2;
    return calculateAngle(averageCount);
}
