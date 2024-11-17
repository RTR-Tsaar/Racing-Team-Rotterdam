/*
 * motorControl.hpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#ifndef INC_MOTORCONTROL_HPP_
#define INC_MOTORCONTROL_HPP_

#include "stm32f1xx_hal.h"
#include "main.hpp"

class MotorControl {
private:
    TIM_HandleTypeDef *pwmTimer;
    uint32_t pwmChannel;
    GPIO_TypeDef *dirPort;
    uint16_t dirPin;

    const int maxPwmValue;
    const int speedIncrement;
    const int rampDelayMs;

    void setSpeedAndDirection(int speed, int direction);
    void rampToSpeed(int targetSpeed, int direction);

public:
    // Constructor
    MotorControl(TIM_HandleTypeDef *timer, uint32_t pwmCh, GPIO_TypeDef *gpioPort, uint16_t gpioPin,
                 int maxPwm = 1000, int increment = 10, int rampDelay = 5);

    // Public interface
    void steerToAngle(int currentAngle, int targetAngle);
    void stopMotorGradually();
};



#endif /* INC_MOTORCONTROL_HPP_ */
