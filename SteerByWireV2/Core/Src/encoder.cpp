/*
 * encoder.cpp
 *
 *  Created on: Nov 21, 2024
 *      Author: rolan
 */
 
#include "encoder.hpp"
#include "cmath"


Encoder::Encoder(MotorControl* motorControl, CurrentSensor* currentSensor, TIM_HandleTypeDef* enc1, TIM_HandleTypeDef* enc2, uint16_t maxCount, float countsPerRevolution)
    : motorControl(motorControl), currentSensor(currentSensor), encoder1(enc1), encoder2(enc2), maxEncoderCount(maxCount), countsPerRev(countsPerRevolution) {
    // Constructor
};

uint32_t Encoder::readEncoder(TIM_HandleTypeDef* enc) {
    // Reads encoder data
    return __HAL_TIM_GET_COUNTER(enc);
}

void Encoder::resetEncoderCount(TIM_HandleTypeDef* enc){
	__HAL_TIM_SET_COUNTER(enc, 0);
}

void Encoder::calibrateEncoder(TIM_HandleTypeDef* enc, float stallCurrent){

	float motorCurrent = currentSensor->getCurrent();

	bool stalling = false; //Checks if the steering motor is stalled. We need this to know when the steering system has reached full lock

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	while(!stalling){
		motorControl->setDutyCycle(3000);

		if (motorCurrent >= stallCurrent) {
			resetEncoderCount(enc);
			motorControl->setDutyCycle(0);
			stalling = true;
		}
	}

	stalling = false;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	while(!stalling){
		motorControl->setDutyCycle(3000);
		if (motorCurrent >= stallCurrent) {
			maxEncoderCount = __HAL_TIM_GET_COUNTER(enc);
			motorControl->setDutyCycle(0);
			stalling = true;
		}
	}
}

int16_t Encoder::calculateAngle(uint16_t count) {
    // Converts encoder count into a tenth of a degree in steer angle
	// Map count to angle within -300 to +300 (tenths of a degree)
	int32_t scaledCount = static_cast<int32_t>(count) * 600; // Scale by total range (60.0 degrees * 10)
	int32_t angle = scaledCount / maxEncoderCount - 300;     // Subtract 300 to map to range [-300, +300]
	return static_cast<int16_t>(angle);
}


//This bit of code is only meant to be used when you have access to 2 encoders on the car
int16_t Encoder::encoderCompare() {
    uint32_t encoder1_count = readEncoder(encoder1);
    uint32_t encoder2_count = readEncoder(encoder2);

    int32_t deficit = static_cast<int32_t>(encoder1_count) - static_cast<int32_t>(encoder2_count);

    if (std::abs(deficit) > 1) {
        // Return error flag if counts are inconsistent
        return -1;
    }

    uint32_t averageCount = (encoder1_count + encoder2_count) / 2;
    return calculateAngle(averageCount);
}
