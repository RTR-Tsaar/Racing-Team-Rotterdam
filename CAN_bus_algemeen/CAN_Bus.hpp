/*
 * CAN_Bus.hpp
 *
 *  Created on: Nov 4, 2024
 *      Author: Dennis Boekholtz
 */

#ifndef INC_CAN_BUS_HPP_
#define INC_CAN_BUS_HPP_

#include "stm32f1xx_hal.h"
#include "main.hpp"

class CANBus {

public:
    // Constructor
    CANBus();
    // Function to initialize the CAN bus
    void start(CAN_HandleTypeDef* hcan);
    void transmit(CAN_HandleTypeDef* hcan, uint8_t TxData[8], uint16_t id);  // Transmit data
    void configureFilter(CAN_HandleTypeDef* hcan, uint16_t ID, uint16_t Mask, uint8_t filterBank, uint8_t slaveFilterBank);  // Configure CAN filters
    void error(CAN_HandleTypeDef* hcan, uint16_t id);  // Transmit data


private:
    CAN_TxHeaderTypeDef TxHeader;  // CAN transmit header
    uint32_t TxMailbox;  // Mailbox for CAN transmission

};


#endif /* INC_CAN_BUS_HPP_ */
