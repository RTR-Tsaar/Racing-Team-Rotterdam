/*
 * motorControl.hpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#ifndef INC_MOTORCONTROL_HPP_
#define INC_MOTORCONTROL_HPP_

#include "stm32f1xx_hal.h"

class MotorControl {
public:
    // Constructor
    MotorControl(TIM_HandleTypeDef* timer, uint32_t channel);

    // Starts PWM
    void start();

    // Steers to a target angle using PID control
    void steerToAngle(uint8_t currentAngle, uint8_t targetAngle);

private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_PinState lastDirection;

    // PID parameters
    float kp;
    float ki;
    float kd;

    float integral;
    float previousError;

    const uint16_t MAX_PWM = 65535; // Maximum PWM value
    const uint16_t MIN_PWM = 0;     // Minimum PWM value

    // Sets the PWM duty cycle
    void setDutyCycle(uint16_t dutyCycle);

    // PID controller for calculating PWM
    int32_t calculatePID(uint8_t targetAngle, uint8_t currentAngle);
};

#endif /* INC_MOTORCONTROL_HPP_ */
