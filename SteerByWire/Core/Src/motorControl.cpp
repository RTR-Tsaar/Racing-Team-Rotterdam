/*
 * motorControl.cpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#include "motorControl.hpp"

// Constructor
MotorControl::MotorControl(TIM_HandleTypeDef* timer, uint32_t channel)
    : htim(timer), channel(channel), lastDirection(GPIO_PIN_SET),
      kp(1.0f), ki(0.1f), kd(0.01f), integral(0), previousError(0) {}

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
    float error = (float)targetAngle - (float)currentAngle;

    // PID calculations
    integral += error;
    float derivative = error - previousError;
    float output = (kp * error) + (ki * integral) + (kd * derivative);

    // Constrain output to valid range
    if (output > MAX_OUTPUT) output = MAX_OUTPUT;
    if (output < MIN_OUTPUT) output = MIN_OUTPUT;

    previousError = error;
    return static_cast<int32_t>(output);
}

// Steers to target angle using PID
void MotorControl::steerToAngle(uint8_t currentAngle, uint8_t targetAngle) {
    // Calculate PID output
    int32_t pidOutput = calculatePID(targetAngle, currentAngle);

    // Determine direction and apply PWM
    if (pidOutput < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Change to one direction
        setDutyCycle(static_cast<uint16_t>(-pidOutput));      // Use absolute value of PID
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // Change to the other direction
        setDutyCycle(static_cast<uint16_t>(pidOutput));
    }
}
