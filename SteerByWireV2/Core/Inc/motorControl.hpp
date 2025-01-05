/*
 * motorControl.hpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#ifndef INC_MOTORCONTROL_HPP
#define INC_MOTORCONTROL_HPP

#include "stm32f1xx_hal.h"

class MotorControl {
public:
    MotorControl(TIM_HandleTypeDef* timer, uint32_t channel);
    void start();
    void setDutyCycle(uint16_t dutyCycle);
    void toggleDirection();
    void steerToAngle(uint16_t currentAngle, uint16_t targetAngle);
    bool isTargetReached(uint16_t currentAngle, uint16_t targetAngle);

private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_PinState lastDirection;
    float integral;
    float previousError;

    int16_t calculatePID(uint16_t targetAngle, uint16_t currentAngle);


    // Fixed PID constants
    static constexpr float KP = 1.0f; // Proportional gain
    static constexpr float KI = 0.1f; // Integral gain
    static constexpr float KD = 0.01f; // Derivative gain

    // Other constants
    static constexpr float MAX_OUTPUT = 65535.0f;
    static constexpr float MIN_OUTPUT = 0.0f;
    static constexpr float MAX_INTEGRAL = 500.0f;
    static constexpr float MIN_INTEGRAL = -500.0f;
    static constexpr int DEAD_ZONE_THRESHOLD = 10;
    static constexpr int ACCEPTABLE_ERROR_MARGIN = 2;
};

#endif // INC_MOTORCONTROL_HPP
