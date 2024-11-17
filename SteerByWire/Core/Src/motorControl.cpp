/*
 * motorControl.cpp
 *
 *  Created on: Nov 15, 2024
 *      Author: rolan
 */

#include <motorControl.hpp>
#include <cstdlib>

// Constructor
MotorControl::MotorControl(TIM_HandleTypeDef *timer, uint32_t pwmCh, GPIO_TypeDef *gpioPort, uint16_t gpioPin,
                           int maxPwm, int increment, int rampDelay)
    : pwmTimer(timer), pwmChannel(pwmCh), dirPort(gpioPort), dirPin(gpioPin),
      maxPwmValue(maxPwm), speedIncrement(increment), rampDelayMs(rampDelay) {
    // Start PWM
    HAL_TIM_PWM_Start(pwmTimer, pwmChannel);
}

// Private method: Set speed and direction
void MotorControl::setSpeedAndDirection(int speed, int direction) {
    // Set direction
    HAL_GPIO_WritePin(dirPort, dirPin, (direction == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Set PWM duty cycle
    __HAL_TIM_SET_COMPARE(pwmTimer, pwmChannel, speed);
}

// Private method: Ramp to target speed
void MotorControl::rampToSpeed(int targetSpeed, int direction) {
    int currentSpeed = __HAL_TIM_GET_COMPARE(pwmTimer, pwmChannel);

    if (currentSpeed < targetSpeed) {
        for (int speed = currentSpeed; speed <= targetSpeed; speed += speedIncrement) {
            setSpeedAndDirection(speed, direction);
            HAL_Delay(rampDelayMs);
        }
    } else if (currentSpeed > targetSpeed) {
        for (int speed = currentSpeed; speed >= targetSpeed; speed -= speedIncrement) {
            setSpeedAndDirection(speed, direction);
            HAL_Delay(rampDelayMs);
        }
    }
}

// Public method: Stop motor gradually
void MotorControl::stopMotorGradually() {
    int currentSpeed = __HAL_TIM_GET_COMPARE(pwmTimer, pwmChannel);

    for (int speed = currentSpeed; speed >= 0; speed -= speedIncrement) {
        setSpeedAndDirection(speed, 0); // Direction doesn't matter when stopping
        HAL_Delay(rampDelayMs);
    }
}

// Public method: Steer to target angle
void MotorControl::steerToAngle(int currentAngle, int targetAngle) {
    int direction;

    // Determine direction
    if (targetAngle > currentAngle) {
        direction = 1; // Right
    } else if (targetAngle < currentAngle) {
        direction = 0; // Left
    } else {
        stopMotorGradually(); // No steering needed
        return;
    }

    // Calculate desired speed based on angle difference
    int angleDiff = abs(targetAngle - currentAngle);
    int targetSpeed = (angleDiff > 30) ? maxPwmValue : (maxPwmValue * angleDiff / 30);

    // Ramp up to target speed
    rampToSpeed(targetSpeed, direction);

    // Maintain speed for steering
    HAL_Delay(500); // Simulate the time it takes to steer (adjust as needed)

    // Gradually slow down to stop
    stopMotorGradually();
}
