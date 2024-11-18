/*
 * motorControl.cpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#include "motorControl.hpp"

// Constructor
MotorControl::MotorControl(TIM_HandleTypeDef* timer, uint32_t channel)
    : htim(timer), channel(channel), lastDirection(GPIO_PIN_RESET) {}

// Starts PWM
void MotorControl::start() {
    HAL_TIM_PWM_Start(htim, channel);
}

// Sets steer speed
void MotorControl::setDutyCycle(uint16_t dutyCycle) {
    __HAL_TIM_SET_COMPARE(htim, channel, dutyCycle);
}

// Ramps up steer speed to prevent jerking action
void MotorControl::rampDutyCycle(uint16_t start, uint16_t end, uint16_t delayMs) {
    for (uint16_t duty = start; duty <= end; duty += 10) {
        setDutyCycle(duty);
        HAL_Delay(delayMs);
    }
}

// Steers to target angle; ramps up only when changing direction and maintains speed afterward
void MotorControl::steerToAngle(uint8_t currentAngle, uint8_t targetAngle) {
    GPIO_PinState currentDirection;

    if (currentAngle > targetAngle) {
        currentDirection = GPIO_PIN_RESET;
    } else if (currentAngle < targetAngle) {
        currentDirection = GPIO_PIN_SET;
    } else {
        setDutyCycle(0); // Stop steering if the current angle matches the target angle
        return;
    }

    // Check if direction has changed
    if (currentDirection != lastDirection) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, currentDirection); // Change direction
        rampDutyCycle(0, 6553, 1); // Ramp up speed after changing direction
        lastDirection = currentDirection; // Update the last direction
    } else {
        setDutyCycle(6553); // Maintain stable speed if direction has not changed
    }
}
