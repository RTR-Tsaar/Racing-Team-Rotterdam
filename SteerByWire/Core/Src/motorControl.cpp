/*
 * motorControl.cpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#include "motorControl.hpp"
#include <cmath>

// Define constants
#define MAX_OUTPUT 1000
#define MIN_OUTPUT 0
#define MAX_INTEGRAL 500.0f
#define MIN_INTEGRAL -500.0f
#define DEAD_ZONE_THRESHOLD 10
#define ACCEPTABLE_ERROR_MARGIN 2

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
    const float dt = 0.01f; // Fixed dt in seconds (10ms)

    // Error calculation
    float error = (float)targetAngle - (float)currentAngle;

    // Integral windup prevention
    integral += error * dt;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

    // Derivative calculation
    float derivative = (error - previousError) / dt;

    // PID calculations
    float output = (kp * error) + (ki * integral) + (kd * derivative);

    // Constrain output to valid range
    if (output > MAX_OUTPUT) output = MAX_OUTPUT;
    if (output < MIN_OUTPUT) output = MIN_OUTPUT;

    // Update previous error for the next iteration
    previousError = error;

    return static_cast<int32_t>(output);
}

// Steers to target angle using PID
void MotorControl::steerToAngle(uint8_t currentAngle, uint8_t targetAngle) {
    // Calculate PID output
    int32_t pidOutput = calculatePID(targetAngle, currentAngle);

    // Apply dead zone to prevent unnecessary motor activation
    if (abs(pidOutput) < DEAD_ZONE_THRESHOLD) {
        setDutyCycle(0);
        return;
    }

    // Determine direction and apply PWM
    if (pidOutput < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Set direction
        setDutyCycle(static_cast<uint16_t>(-pidOutput));      // Use absolute value of PID
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // Set direction
        setDutyCycle(static_cast<uint16_t>(pidOutput));
    }
}

// Dynamically set PID parameters
void MotorControl::setPIDParameters(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

// Check if target angle is reached
bool MotorControl::isTargetReached(uint8_t currentAngle, uint8_t targetAngle) {
    float error = (float)targetAngle - (float)currentAngle;
    return abs(error) < ACCEPTABLE_ERROR_MARGIN;
}
