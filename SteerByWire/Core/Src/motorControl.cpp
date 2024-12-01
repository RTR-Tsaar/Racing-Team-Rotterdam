/*
 * motorControl.cpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#include "motorControl.hpp"
#include <cmath>

// Constructor
MotorControl::MotorControl(TIM_HandleTypeDef* timer, uint32_t channel)
    : htim(timer), channel(channel), lastDirection(GPIO_PIN_SET),
      integral(0.0f), previousError(0.0f) {}

// Starts PWM
void MotorControl::start() {
    HAL_TIM_PWM_Start(htim, channel);
}

// Sets steer speed (duty cycle)
void MotorControl::setDutyCycle(uint16_t dutyCycle) {
    __HAL_TIM_SET_COMPARE(htim, channel, dutyCycle);
}

// PID controller implementation
int32_t MotorControl::calculatePID(uint8_t targetAngle, uint8_t currentAngle) {
    const float dt = 0.01f; // Fixed dt in seconds (10ms)

    // Error calculation
    float error = static_cast<float>(targetAngle - currentAngle);

    // Integral windup prevention
    integral += error * dt;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

    // Derivative calculation
    float derivative = (error - previousError) / dt;

    // PID calculations
    float output = (KP * error) + (KI * integral) + (KD * derivative);

    // Constrain output to valid range
    if (output > MAX_OUTPUT) output = MAX_OUTPUT;
    if (output < MIN_OUTPUT) output = MIN_OUTPUT;


    // Update previous error for the next iteration
    previousError = error;

    return static_cast<int32_t>(output);
}

// Steers to target angle using PID
void MotorControl::steerToAngle(uint8_t currentAngle, uint8_t targetAngle) {
    int32_t pidOutput = calculatePID(targetAngle, currentAngle);

    // Apply dead zone
    if (std::abs(pidOutput) < DEAD_ZONE_THRESHOLD) {
        setDutyCycle(0);
        return;
    }

    // Determine direction and apply PWM
    if (pidOutput < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Reverse direction
        setDutyCycle(static_cast<uint16_t>(-pidOutput));      // Positive duty cycle
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // Forward direction
        setDutyCycle(static_cast<uint16_t>(pidOutput));
    }
}

// Check if target angle is reached
bool MotorControl::isTargetReached(uint8_t currentAngle, uint8_t targetAngle) {
    return std::abs(static_cast<float>(targetAngle - currentAngle)) < ACCEPTABLE_ERROR_MARGIN;
}
