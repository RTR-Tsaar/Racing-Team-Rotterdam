/*
 * motorControl.hpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#ifndef INC_MOTORCONTROL_HPP_
#define INC_MOTORCONTROL_HPP_

#include "stm32f1xx_hal.h"
//#include "main.hpp"

class MotorControl {
public:
    MotorControl(TIM_HandleTypeDef* timer, uint32_t channel);

    void start();
    void rampDutyCycle(uint16_t start, uint16_t end, uint16_t delayMs);
    void setDutyCycle(uint16_t dutyCycle);

    void steerToAngle(uint8_t currentAngle, uint8_t targetAngle);


private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;

};



#endif /* INC_MOTORCONTROL_HPP_ */
