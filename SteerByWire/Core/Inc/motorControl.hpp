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
    int32_t calculatePID(uint8_t targetAngle, uint8_t currentAngle);
    void steerToAngle(uint8_t currentAngle, uint8_t targetAngle);
    void setPIDParameters(float kp_, float ki_, float kd_);
    bool isTargetReached(uint8_t currentAngle, uint8_t targetAngle);

private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_PinState lastDirection;
    float kp, ki, kd;
    float integral;
    float previousError;
};

#endif // INC_MOTORCONTROL_HPP
