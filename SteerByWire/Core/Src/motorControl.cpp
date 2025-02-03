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
    : htim(timer), channel(channel),
      integral(0.0f), previousError(0.0f) {}

// Starts PWM
void MotorControl::start() {
    HAL_TIM_PWM_Start(htim, channel);
}

// Sets steer speed (duty cycle)
void MotorControl::setDutyCycle(uint16_t dutyCycle) {
    __HAL_TIM_SET_COMPARE(htim, channel, dutyCycle);
}
//This method can be used to toggle the direction of the motor
void MotorControl::toggleDirection() {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
}

// PID controller implementation
int MotorControl::calculatePID(int targetAngle, int currentAngle) {
    const float dt = 0.01f; // Fixed dt in seconds (10ms)

    // Error calculation
    int error = targetAngle - currentAngle;

    // Integral windup prevention
    integral += error * dt;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

    // Derivative calculation
    float derivative = (error - previousError) / dt;

    // PID calculations
    float output = (KP * error) + (KI * integral) + (KD * derivative);

    // Update previous error for the next iteration
    previousError = error;

    return static_cast<int>(output);
}

// Steers to target angle using PID
void MotorControl::steerToAngle(int currentAngle, int targetAngle) {
    int pidOutput = calculatePID(targetAngle, currentAngle);

    int ampedOutput = pidOutput * 10;

    if (ampedOutput > MAX_OUTPUT) ampedOutput = MAX_OUTPUT;
    if (ampedOutput < MIN_OUTPUT) ampedOutput = MIN_OUTPUT;



    // Determine direction and apply PWM
    if (ampedOutput < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Reverse direction
        setDutyCycle(static_cast<uint16_t>(-ampedOutput));      // Positive duty cycle
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // Forward direction
        setDutyCycle(static_cast<uint16_t>(ampedOutput));
    }
}

