/*
 * motorControl.cpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#include "motorControl.hpp"

//constructor
MotorControl::MotorControl(TIM_HandleTypeDef* timer, uint32_t channel)
    : htim(timer), channel(channel) {}

//starts PWM
void MotorControl::start() {
    HAL_TIM_PWM_Start(htim, channel);
}

//sets steer speed
void MotorControl::setDutyCycle(uint16_t dutyCycle) {
    __HAL_TIM_SET_COMPARE(htim, channel, dutyCycle);
}

//ramps up steer speed to prevent jerking action
void MotorControl::rampDutyCycle(uint16_t start, uint16_t end, uint16_t delayMs) {
        for (uint16_t duty = start; duty <= end; duty+=10) {
            setDutyCycle(duty);
            HAL_Delay(delayMs);
        }
//		setDutyCycle(0);
}


//steers to target angle, when target angle is reached it stops steering
void MotorControl::steerToAngle(uint8_t currentAngle, uint8_t targetAngle){
	if (currentAngle>targetAngle) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		rampDutyCycle(0, 6553, 1);
	}
	else if (currentAngle<targetAngle){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		rampDutyCycle(0, 6553, 1);
	}
	else {
		setDutyCycle(0);
	}
}
