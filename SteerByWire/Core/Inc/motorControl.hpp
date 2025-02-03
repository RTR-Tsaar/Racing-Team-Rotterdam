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
    void steerToAngle(int currentAngle, int targetAngle);

private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
//    GPIO_PinState lastDirection;
    float integral;
    float previousError;

    int calculatePID(int targetAngle, int currentAngle);


    // Fixed PID constants
    static constexpr float KP = 20.0f; // Proportional gain
    static constexpr float KI = 0.5f; // Integral gain
    static constexpr float KD = 1.0f; // Derivative gain

    // Other constants
    static constexpr float MAX_OUTPUT = 65535.0f;
    static constexpr float MIN_OUTPUT = -MAX_OUTPUT;
    static constexpr float MAX_INTEGRAL = 500.0f;
    static constexpr float MIN_INTEGRAL = 0.0f;
};

#endif // INC_MOTORCONTROL_HPP
