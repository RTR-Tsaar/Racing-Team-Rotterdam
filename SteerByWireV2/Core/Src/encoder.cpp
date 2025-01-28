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

//This method reads out an encoders value
uint32_t Encoder::readEncoder(TIM_HandleTypeDef* enc) {
    // Reads encoder data
    return __HAL_TIM_GET_COUNTER(enc);
}

//This method is used to set the encoder reading to zero.
void Encoder::resetEncoderCount(TIM_HandleTypeDef* enc){
	__HAL_TIM_SET_COUNTER(enc, 0);
}

//This method calibrates the encoder to make sure it accuratly reads the correct steering angle.
void Encoder::calibrateEncoder(TIM_HandleTypeDef* enc, float stallCurrent){

	int lowSpeed = 10000; //This value decides the speed at which the motor will turn. Max speed is 65535.
	float motorCurrent;
	int encoderReading;

	bool stalling = false; //Checks if the steering motor is stalled. We need this to know when the steering system has reached full lock

	currentSensor->start();//Starts the adc channel of the current sensor

	motorControl->setDutyCycle(lowSpeed);

	while(!stalling){		//As long as the motor isn't stalling it will remain in this loop.
		motorCurrent = currentSensor->getCurrent(); //Reads out the current

		//If th motorcurrent is equal or higher than the stall current, it means the motor is stalling and has reached full lock.
		if (motorCurrent >= stallCurrent) {
			resetEncoderCount(enc);	//When full lock is reached the encoder is reset. This way we know that an encoder reading of 0 means full lock to the left.
			motorControl->setDutyCycle(0);//Stops the motor from turning
			stalling = true;	//Allows the code to continue out of the while loop.
		}
		HAL_Delay(100);

	}

	stalling = false;

	motorControl->toggleDirection(); //Changes direction of the motor
	motorControl->setDutyCycle(lowSpeed);	//Allows motor to turn at low speed

	//This loop does the same is the previous one. The main difference is it records the position at full lock, instead of setting it to zero.
	while(!stalling){
		motorCurrent = currentSensor->getCurrent();

		if (motorCurrent >= stallCurrent) {
			maxEncoderCount = __HAL_TIM_GET_COUNTER(enc); //records maximum encoder rotation to give us the full range of the encoder.
			motorControl->setDutyCycle(0);
			stalling = true;
		}
		HAL_Delay(100);
	}
	currentSensor->stop();//Turns of the adc channel for the current sensor as we no longer need it.
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
